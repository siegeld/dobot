# OpenVLA-OFT inference server.
#
# Two build flavors controlled by the OFT_MODE build arg:
#   - mock: CPU-only, tiny image. Returns random action chunks. For offline dev.
#   - real: CUDA 12.8 + PyTorch nightly for Blackwell SM 120. Pulls OFT deps.
#
# Build:
#   docker build --build-arg OFT_MODE=mock -t oft-server:mock .
#   docker build --build-arg OFT_MODE=real -t oft-server:real .

ARG OFT_MODE=mock

# ── Mock base ────────────────────────────────────────────────────
FROM python:3.11-slim AS base-mock
ENV PIP_NO_CACHE_DIR=1
WORKDIR /app
COPY requirements.txt .
RUN pip install -r requirements.txt

# ── Real base (CUDA + OFT) ───────────────────────────────────────
# Using the Nvidia PyTorch container gives us a working CUDA 12.8 + torch with
# SM 120 (Blackwell) support without wrestling with pip wheels.
FROM nvcr.io/nvidia/pytorch:25.01-py3 AS base-real
ENV PIP_NO_CACHE_DIR=1
WORKDIR /app
COPY requirements.txt .
RUN pip install -r requirements.txt \
 && pip install \
      transformers==4.44.2 \
      accelerate>=0.33 \
      peft>=0.11 \
      timm \
      bitsandbytes \
      tokenizers \
      sentencepiece
# OFT / prismatic are typically installed from source. Install at runtime via
# a bind mount so we can iterate on the training repo without rebuilding:
#   docker run -v /path/to/openvla-oft:/opt/openvla-oft ...
# entrypoint.sh will `pip install -e /opt/openvla-oft` if present.

# ── Final stage (picks one based on OFT_MODE) ────────────────────
FROM base-${OFT_MODE} AS final
COPY server.py mock_server.py entrypoint.sh /app/
RUN chmod +x /app/entrypoint.sh
EXPOSE 7071
ENTRYPOINT ["/app/entrypoint.sh"]
