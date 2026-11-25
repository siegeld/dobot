"""The dashboard's sign-in gate (web/auth.py): every API route and websocket needs
the session cookie, the pages serve the login page without one, the break-glass
admin signs in, and the SSO endpoint keeps homelab-auth's 404/401 semantics. The
protocol itself (NTLM and foreign-realm refusals) is tested in homelab-auth.

Run in the image (ROS is importable there):
  docker compose run --rm --no-deps -T -v $PWD/tests:/tests:ro -e PYTHONPATH=/dobot-ros \
    -e SESSION_SECRET=test -e ADMIN_PASSWORD=test-admin-pw -e AD_ENABLED=false -e SSO_KEYTAB= \
    dobot bash -lc 'pip3 -q install --break-system-packages httpx2 && cd / && python3 -m pytest /tests/web -q -p no:cacheprovider'
(starlette 1.0's TestClient wants httpx2, which the image does not ship: a test-only dependency.)
"""
from __future__ import annotations

import os

import pytest

os.environ.setdefault("SESSION_SECRET", "test-session-secret")
os.environ.setdefault("ADMIN_PASSWORD", "test-admin-pw")
os.environ.setdefault("AD_ENABLED", "false")
os.environ.setdefault("SSO_KEYTAB", "")
os.environ.pop("AUTH_DISABLED", None)

from fastapi.testclient import TestClient  # noqa: E402
from starlette.websockets import WebSocketDisconnect  # noqa: E402

from dobot_ros.web import auth  # noqa: E402
from dobot_ros.web.server import app  # noqa: E402


@pytest.fixture(scope="module")
def client():
    # no `with`: the lifespan would start ROS. The gate and the auth routes need none of it.
    # https base: the cookie is Secure and a plain-http client would never send it back.
    return TestClient(app, base_url="https://testserver")


def test_api_needs_a_session(client):
    r = client.get("/api/status")
    assert r.status_code == 401 and r.json()["detail"] == "not authenticated"


def test_pages_serve_the_login_gate(client):
    for path in ("/", "/pendant", "/spacemouse"):
        r = client.get(path)
        assert r.status_code == 200 and "Sign in with your domain account" in r.text, path
    # the dashboard's own assets and liveness stay public
    assert client.get("/healthz").status_code == 200
    assert client.get("/static/css/dashboard.css").status_code == 200


def test_websocket_without_session_closes_4401(client):
    with pytest.raises(WebSocketDisconnect) as e:
        with client.websocket_connect("/ws/state") as ws:
            ws.receive_text()
    assert e.value.code == 4401


def test_login_bad_password_and_me(client):
    assert client.post("/auth/login", json={"account": "admin", "password": "nope"}).status_code == 401
    # AD is off: a domain account cannot sign in at all
    assert client.post("/auth/login", json={"account": "david", "password": "x"}).status_code == 401
    assert client.get("/auth/me").status_code == 401


def test_admin_signs_in_and_the_gate_opens(client):
    r = client.post("/auth/login", json={"account": "admin", "password": "test-admin-pw", "remember": True})
    assert r.status_code == 200 and r.json() == {"name": "admin", "email": None, "role": "admin"}
    cookie = r.cookies.get(auth.COOKIE_NAME)
    assert cookie and auth.read_session(cookie)["sub"] == "admin"
    hdr = {"Cookie": f"{auth.COOKIE_NAME}={cookie}"}
    assert client.get("/auth/me", headers=hdr).json()["name"] == "admin"
    assert client.get("/api/status", headers=hdr).status_code == 200
    assert "<title>Dobot CR5 Controller</title>" in client.get("/", headers=hdr).text
    assert client.post("/auth/logout", headers=hdr).status_code == 200
    # a tampered cookie is no session
    assert client.get("/api/status", headers={"Cookie": f"{auth.COOKIE_NAME}={cookie[:-3]}xyz"}).status_code == 401


def test_sso_404_when_not_configured(client, monkeypatch):
    monkeypatch.setenv("SSO_KEYTAB", "")
    assert client.get("/auth/sso").status_code == 404


def test_sso_challenges_then_mints_a_session(client, monkeypatch, tmp_path):
    from homelab_auth import Principal, ad, kerberos
    kt = tmp_path / "http.keytab"
    kt.write_bytes(b"\x05\x02")
    monkeypatch.setenv("SSO_KEYTAB", str(kt))
    r = client.get("/auth/sso")
    assert r.status_code == 401 and r.headers["www-authenticate"] == "Negotiate"

    monkeypatch.setattr(kerberos, "accept", lambda cfg, tok: ("siegeld@SIEGEL.COM", "c2VydmVy"))
    seen = {}

    def fake_lookup(cfg, principal):
        seen["principal"], seen["groups"] = principal, cfg.admin_groups
        return Principal(upn="siegeld@siegel.com", display_name="David Siegel", groups=["Domain Admins"],
                         sam="siegeld", email="siegeld@siegel.com")
    monkeypatch.setattr(ad, "lookup", fake_lookup)
    r = client.get("/auth/sso", headers={"Authorization": "Negotiate YWJj"})
    assert r.status_code == 200, r.text
    assert seen["principal"] == "siegeld@SIEGEL.COM" and "Domain Admins" in seen["groups"]
    assert r.headers["www-authenticate"] == "Negotiate c2VydmVy"
    assert auth.read_session(r.cookies.get(auth.COOKIE_NAME))["sub"] == "siegeld"


def test_sso_reserved_local_name_is_refused(client, monkeypatch, tmp_path):
    from homelab_auth import Principal, ad, kerberos
    kt = tmp_path / "http.keytab"
    kt.write_bytes(b"\x05\x02")
    monkeypatch.setenv("SSO_KEYTAB", str(kt))
    monkeypatch.setattr(kerberos, "accept", lambda cfg, tok: ("admin@SIEGEL.COM", None))
    monkeypatch.setattr(ad, "lookup", lambda cfg, p: Principal(upn="admin@siegel.com", display_name="admin",
                                                                groups=["Domain Admins"], sam="admin"))
    r = client.get("/auth/sso", headers={"Authorization": "Negotiate YWJj"})
    assert r.status_code == 401 and auth.COOKIE_NAME not in r.cookies


def test_auth_disabled_opens_the_bench(client, monkeypatch):
    client.cookies.clear()  # the SSO test above left a session in the jar
    monkeypatch.setenv("AUTH_DISABLED", "true")
    assert client.get("/api/status").status_code == 200
    assert client.get("/auth/me").json()["name"] == "open bench"
