"""Human sign-in for the dashboard: Active Directory, Kerberos browser SSO, and a
break-glass local admin (homelab-system-ui §7). The directory login, the ticket
check, the admin-group gate and the Negotiate protocol semantics are the shared
`homelab-auth` package; this module maps `.env` onto it, mints the dashboard's own
session cookie, and gates every route and websocket behind that cookie.

Policy: a robot arm that moves is operated by Domain Admins. A good password or a
valid ticket is necessary but not sufficient; `AD_ADMIN_GROUP` membership is the
gate, enforced by the package for both paths. The local `admin` account (password in
`.env`) always works, so AD or the keytab being down never locks the arm's console.

Public without a session: `/auth/*`, `/static/*` (the dashboard's own JS/CSS, nothing
secret in it) and `/healthz`. The three HTML pages serve the login page instead; every
`/api/*` call answers 401; a websocket is accepted and closed with code 4401 so the
page can reload into the login gate.
"""
from __future__ import annotations

import asyncio
import logging
import os
import time
from collections import deque
from http.cookies import SimpleCookie
from pathlib import Path

from fastapi import APIRouter, Depends, Header, HTTPException, Request, Response
from fastapi.responses import FileResponse, JSONResponse
from homelab_auth import AuthConfig, AuthError, ad
from homelab_auth.fastapi import SsoResult, negotiate, reply_headers
from itsdangerous import BadSignature, URLSafeTimedSerializer
from pydantic import BaseModel

log = logging.getLogger("dobot.auth")

LOCAL_ADMIN = "admin"
COOKIE_NAME = "dobot_session"
SESSION_TTL_S = 12 * 3600          # a shift; SSO renews silently on the next visit
REMEMBER_TTL_S = 30 * 24 * 3600    # the "stay signed in" box
STATIC_DIR = Path(__file__).parent / "static"


def _env(key: str, default: str = "") -> str:
    return os.environ.get(key, default)


def _bool(key: str, default: bool) -> bool:
    v = os.environ.get(key)
    return default if v is None else v.strip().lower() in ("1", "true", "yes", "on")


def disabled() -> bool:
    """AUTH_DISABLED=true runs the dashboard open, for a bench with no directory.
    Logged loudly at startup; never the shipped configuration."""
    return _bool("AUTH_DISABLED", False)


def auth_config() -> AuthConfig:
    pw = _env("ADMIN_PASSWORD")
    return AuthConfig(
        ad_enabled=_bool("AD_ENABLED", True),
        ad_server=_env("AD_SERVER", "ldap://dcrbr1.siegel.com"),
        ad_domain=_env("AD_DOMAIN", "siegel.com"),
        ad_base_dn=_env("AD_BASE_DN", "DC=siegel,DC=com"),
        admin_groups=AuthConfig.split_groups(_env("AD_ADMIN_GROUP", "Domain Admins")),
        admin_required=True,
        ad_bind_dn=_env("AD_BIND_DN"),
        ad_bind_password=_env("AD_BIND_PASSWORD"),
        local_admin_user=LOCAL_ADMIN if pw else "",
        local_admin_password=pw,
        sso_keytab=_env("SSO_KEYTAB"),
        sso_realm=_env("SSO_REALM", "SIEGEL.COM"),
    )


def sso_configured() -> bool:
    kt = _env("SSO_KEYTAB")
    return bool(kt) and os.path.exists(kt)


# ── sessions ───────────────────────────────────────────────────────────────
def _serializer() -> URLSafeTimedSerializer:
    secret = _env("SESSION_SECRET")
    if not secret:
        raise RuntimeError("SESSION_SECRET is not set: refusing to run the dashboard with no sign-in "
                           "(set it in .env, or AUTH_DISABLED=true for an open bench)")
    return URLSafeTimedSerializer(secret, salt="dobot-session")


def make_session(*, sub: str, name: str, email: str | None, role: str, ttl: int) -> str:
    return _serializer().dumps({"sub": sub, "name": name, "email": email, "role": role, "ttl": ttl})


def read_session(token: str | None) -> dict | None:
    if not token:
        return None
    try:
        data, ts = _serializer().loads(token, max_age=REMEMBER_TTL_S, return_timestamp=True)
    except BadSignature:
        return None
    if time.time() > ts.timestamp() + int(data.get("ttl", SESSION_TTL_S)):
        return None
    return data


def _cookie_token(headers: list[tuple[bytes, bytes]]) -> str | None:
    raw = b"; ".join(v for k, v in headers if k == b"cookie")
    if not raw:
        return None
    c = SimpleCookie()
    try:
        c.load(raw.decode("latin-1"))
    except Exception:  # noqa: BLE001 — a malformed cookie header is just "no session"
        return None
    m = c.get(COOKIE_NAME)
    return m.value if m else None


# ── the gate ───────────────────────────────────────────────────────────────
PUBLIC_PREFIXES = ("/auth/", "/static/")
PUBLIC_PATHS = {"/healthz", "/auth"}
PAGES = {"/", "/pendant", "/spacemouse"}
_NO_CACHE = {"Cache-Control": "no-cache, no-store, must-revalidate", "Pragma": "no-cache", "Expires": "0"}


class AuthGate:
    """Pure ASGI middleware: one cookie check in front of HTTP and websocket routes."""

    def __init__(self, app) -> None:
        self.app = app

    async def __call__(self, scope, receive, send):
        if scope["type"] not in ("http", "websocket") or disabled():
            return await self.app(scope, receive, send)
        path: str = scope["path"]
        if path in PUBLIC_PATHS or path.startswith(PUBLIC_PREFIXES):
            return await self.app(scope, receive, send)
        principal = read_session(_cookie_token(scope.get("headers", [])))
        if principal:
            scope.setdefault("state", {})["principal"] = principal
            return await self.app(scope, receive, send)
        if scope["type"] == "websocket":
            # accept, then close with our code: a refused handshake would reach the
            # browser as a bare 1006 and the page could not tell "sign in" from "down"
            await send({"type": "websocket.accept"})
            await send({"type": "websocket.close", "code": 4401, "reason": "not authenticated"})
            return
        if path in PAGES:
            return await FileResponse(str(STATIC_DIR / "login.html"), headers=_NO_CACHE)(scope, receive, send)
        return await JSONResponse({"detail": "not authenticated"}, status_code=401)(scope, receive, send)


# ── routes ─────────────────────────────────────────────────────────────────
router = APIRouter(prefix="/auth", tags=["auth"])
_negotiate = negotiate(auth_config)


class Throttle:
    """Wrong passwords per account and per address: 10 in 10 minutes locks that key."""
    LIMIT, WINDOW_S = 10, 600

    def __init__(self) -> None:
        self._fails: dict[str, deque] = {}

    def _bucket(self, key: str) -> deque:
        q = self._fails.setdefault(key, deque())
        cutoff = time.time() - self.WINDOW_S
        while q and q[0] < cutoff:
            q.popleft()
        return q

    def blocked(self, account: str, ip: str) -> bool:
        return len(self._bucket(f"a:{account}")) >= self.LIMIT or len(self._bucket(f"ip:{ip}")) >= self.LIMIT

    def fail(self, account: str, ip: str) -> None:
        now = time.time()
        self._bucket(f"a:{account}").append(now)
        self._bucket(f"ip:{ip}").append(now)

    def clear(self, account: str) -> None:
        self._fails.pop(f"a:{account}", None)


throttle = Throttle()


def _client_ip(request: Request) -> str:
    # jupiter's nginx sits in front and sets X-Real-IP; the socket peer is 127.0.0.1
    return request.headers.get("x-real-ip") or (request.client.host if request.client else "?")


class LoginBody(BaseModel):
    account: str
    password: str
    remember: bool = False


class Me(BaseModel):
    name: str
    email: str | None = None
    role: str


def _issue(response: Response, *, sub: str, name: str, email: str | None, remember: bool) -> Me:
    ttl = REMEMBER_TTL_S if remember else SESSION_TTL_S
    token = make_session(sub=sub, name=name, email=email, role="admin", ttl=ttl)
    response.set_cookie(COOKIE_NAME, token, max_age=ttl, httponly=True, samesite="lax", secure=True, path="/")
    return Me(name=name, email=email, role="admin")


async def sso_negotiate(authorization: str | None = Header(None)) -> SsoResult:
    """homelab-auth's Negotiate dependency plus ONE log line when it refuses a
    ticket, so a silent fall-back to the form can be diagnosed from the server."""
    try:
        return await _negotiate(authorization)
    except HTTPException as e:
        if e.status_code == 401 and e.detail != "negotiate":
            log.info("sso refused: %s", e.detail)
        raise


@router.post("/login", response_model=Me)
async def login(body: LoginBody, request: Request, response: Response) -> Me:
    key, ip = body.account.strip().lower(), _client_ip(request)
    if throttle.blocked(key, ip):
        raise HTTPException(status_code=429, detail="too many failed sign-ins; wait ten minutes")
    try:
        user = await asyncio.to_thread(ad.authenticate, auth_config(), body.account.strip(), body.password)
    except AuthError as e:
        not_authorized = str(e).startswith("not authorized")
        if not not_authorized:
            throttle.fail(key, ip)
        log.info("login refused for %s from %s: %s", key, ip, e)
        raise HTTPException(status_code=403 if not_authorized else 401,
                            detail="not authorized for this system (admins only)" if not_authorized
                            else "invalid credentials") from e
    throttle.clear(key)
    log.info("login: %s from %s", key, ip)
    if user.is_local:
        return _issue(response, sub=LOCAL_ADMIN, name=LOCAL_ADMIN, email=None, remember=body.remember)
    return _issue(response, sub=user.sam, name=user.display_name or user.sam, email=user.email, remember=body.remember)


@router.get("/sso", response_model=Me)
async def sso_login(response: Response, r: SsoResult = Depends(sso_negotiate)) -> Me:
    """Browser single sign-on (Kerberos / Negotiate): 404 when no keytab is configured,
    401 + challenge when there is no ticket, 401 without one for every refusal."""
    u = r.user
    if u.sam == LOCAL_ADMIN:
        log.info("sso refused: reserved account name (%s)", r.principal)
        raise HTTPException(status_code=401, detail="reserved account name")
    log.info("sso: signed in %s", r.principal)
    response.headers.update(reply_headers(r))
    return _issue(response, sub=u.sam, name=u.display_name or u.sam, email=u.email, remember=False)


@router.get("/me", response_model=Me)
async def me(request: Request) -> Me:
    p = read_session(request.cookies.get(COOKIE_NAME))
    if p is None:
        if disabled():
            return Me(name="open bench", email=None, role="admin")
        raise HTTPException(status_code=401, detail="not authenticated")
    return Me(name=p["name"], email=p.get("email"), role=p.get("role", "admin"))


@router.post("/logout")
async def logout(response: Response) -> dict:
    response.delete_cookie(COOKIE_NAME, path="/")
    return {"ok": True}
