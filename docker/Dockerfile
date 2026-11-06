# syntax=docker/dockerfile:1
# --------------------------------------------------------------------------- #
# isaac-drone-nav runtime image
#   NVIDIA Isaac Sim 5.1.0 (official container) + Pegasus Simulator v5.1.0
#   + the dronenav package (editable install).
#
# The same image runs on a local Ubuntu workstation, on AWS (g5/g6/g6e) or on
# any Linux host with an RTX-class NVIDIA GPU, a recent driver and the NVIDIA
# Container Toolkit. Build from the repository root:
#
#   docker compose -f docker/docker-compose.yml build
# --------------------------------------------------------------------------- #
ARG ISAACSIM_VERSION=5.1.0
FROM nvcr.io/nvidia/isaac-sim:${ISAACSIM_VERSION}

ARG PEGASUS_VERSION=v5.1.0
ARG UID=1234
ARG GID=1234

ENV ACCEPT_EULA=Y \
    PRIVACY_CONSENT=Y \
    ISAACSIM_PATH=/isaac-sim \
    ISAACSIM_PYTHON=/isaac-sim/python.sh \
    PEGASUS_PATH=/opt/PegasusSimulator \
    PROJECT_ROOT=/workspace/isaac-drone-nav \
    PYTHONUNBUFFERED=1

USER root
RUN apt-get update \
    && apt-get install -y --no-install-recommends git ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# Freeze the scientific stack shipped with Isaac Sim so that no later pip
# install can silently upgrade it (a classic cause of Isaac Sim crashes).
RUN ${ISAACSIM_PYTHON} -m pip freeze \
    | grep -iE '^(numpy|scipy|pyyaml|torch)==' > /opt/isaac-constraints.txt \
    && cat /opt/isaac-constraints.txt

# Pegasus Simulator, pinned to the release matching Isaac Sim 5.1.0.
# Its setup.py reads ISAACSIM_PATH and patches the Isaac Sim kit app file.
RUN git clone --depth 1 --branch ${PEGASUS_VERSION} \
        https://github.com/PegasusSimulator/PegasusSimulator.git ${PEGASUS_PATH} \
    && ${ISAACSIM_PYTHON} -m pip install --no-cache-dir \
        -c /opt/isaac-constraints.txt \
        -e ${PEGASUS_PATH}/extensions/pegasus.simulator

# Project dependencies and editable install of dronenav. At run time the
# repository is bind-mounted over ${PROJECT_ROOT}, so code edits on the host
# are picked up without rebuilding the image.
WORKDIR ${PROJECT_ROOT}
COPY requirements-isaac.txt pyproject.toml README.md LICENSE ./
COPY src ./src
RUN ${ISAACSIM_PYTHON} -m pip install --no-cache-dir -c /opt/isaac-constraints.txt -r requirements-isaac.txt \
    && ${ISAACSIM_PYTHON} -m pip install --no-cache-dir --no-deps -e .

RUN chown -R ${UID}:${GID} ${PEGASUS_PATH} /workspace
USER ${UID}:${GID}

ENTRYPOINT []
CMD ["bash"]
