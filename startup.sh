#!/bin/bash
# Start the Dobot CR5 system: driver + web dashboard
#
# Usage:
#   ./startup.sh          # Start driver + web dashboard
#   ./startup.sh --build  # Rebuild image first, then start
#   ./startup.sh --stop   # Stop everything

set -e
cd "$(dirname "$0")"

RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

info()  { echo -e "${BLUE}[INFO]${NC}  $1"; }
ok()    { echo -e "${GREEN}[OK]${NC}    $1"; }
err()   { echo -e "${RED}[ERR]${NC}   $1"; }

# ── Stop ────────────────────────────────────────────────────
if [ "$1" = "--stop" ]; then
    info "Stopping all services..."
    docker compose down
    ok "All services stopped"
    exit 0
fi

# ── Banner ──────────────────────────────────────────────────
echo -e "${CYAN}"
echo "  ╔══════════════════════════════════════╗"
echo "  ║       Dobot CR5 Control System       ║"
echo "  ║     ROS2 Driver + Web Dashboard      ║"
echo "  ╚══════════════════════════════════════╝"
echo -e "${NC}"

# ── Check .env ──────────────────────────────────────────────
if [ ! -f .env ]; then
    err ".env file not found"
    info "Create one with:  echo 'ROBOT_IP=192.168.5.1' > .env"
    exit 1
fi

source .env
info "Robot IP: ${ROBOT_IP:-not set}"
info "Robot Type: ${ROBOT_TYPE:-cr5}"

# ── Build if requested or first time ────────────────────────
if [ "$1" = "--build" ] || ! docker compose images dobot-driver --quiet 2>/dev/null | grep -q .; then
    info "Building Docker image..."
    docker compose build
    ok "Image built"
fi

# ── Stop existing services ──────────────────────────────────
docker compose down 2>/dev/null || true

# ── Start ───────────────────────────────────────────────────
info "Starting driver + gripper + web dashboard..."
docker compose up -d dobot-driver dobot-gripper dobot-web
ok "Services starting (driver will auto-reconnect when robot is reachable)"

echo ""
echo -e "${GREEN}══════════════════════════════════════${NC}"
echo -e "  ${CYAN}Web Dashboard:${NC}  http://localhost:7070"
echo -e "  ${CYAN}Robot IP:${NC}       ${ROBOT_IP:-192.168.5.1}"
echo -e "${GREEN}══════════════════════════════════════${NC}"
echo ""
info "Logs: docker compose logs -f"
info "Stop: ./startup.sh --stop"

# ── Follow logs ─────────────────────────────────────────────
docker compose logs -f dobot-driver dobot-web
