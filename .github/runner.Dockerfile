# GitHub's runner of the host's gate (.github/workflows/host.yml) as an image here, built and run by
# host/tools/dev/linux.py (run_tests.py --linux), which passes host.yml's own choices as the arguments below:
# its Ubuntu release, its apt packages, its Python. Built of this file, host/requirements.txt and
# host/pyproject.toml alone.
ARG RELEASE=latest
FROM ubuntu:${RELEASE}
ARG PYTHON
ARG APT=""
ARG USER=runner
ARG UID=1001
ENV DEBIAN_FRONTEND=noninteractive LANG=C.UTF-8 LC_ALL=C.UTF-8 RUNNER_TOOL_CACHE=/opt/hostedtoolcache
SHELL ["/bin/bash", "-euo", "pipefail", "-c"]

# The runner's image installs the archive's amd64v3 builds (its job's log: *_amd64v3.deb) - here on a processor
# of x86-64-v3, as glibc's loader tells it, else the baseline's. Then what its image brings that the gate uses -
# a compiler, git, python3 and the libraries the Pythons below link, xvfb (its readme's packages) and the GL
# libraries it brings: PyOpenGL's EGL platform loads libGL, and importing mujoco crashed without it -, and
# host.yml's packages as .github/apt.sh installs them.
RUN if /lib64/ld-linux-x86-64.so.2 --help | grep -q 'x86-64-v3 (supported'; then \
      echo 'APT::Architecture-Variants "amd64v3";' > /etc/apt/apt.conf.d/99enable-amd64v3; fi; \
    apt-get update -qq; \
    apt-get dist-upgrade -y -qq; \
    apt-get install -y -qq --no-install-recommends build-essential ca-certificates curl git python3 sudo xvfb \
      xz-utils; \
    if [ -n "${APT}" ]; then apt-get install -y -qq ${APT}; fi; \
    rm -rf /var/lib/apt/lists/*

# The runner's user: its id, sudo without a password, the tool cache and the workspace its own.
RUN useradd --create-home --uid "${UID}" --shell /bin/bash "${USER}"; \
    echo "${USER} ALL=(ALL) NOPASSWD:ALL" > "/etc/sudoers.d/${USER}"; \
    mkdir -p "${RUNNER_TOOL_CACHE}" "/home/${USER}/work"; \
    chown -R "${USER}:" "${RUNNER_TOOL_CACHE}" "/home/${USER}/work"
USER ${USER}

# The Python setup-python takes there: actions/python-versions' build of this release, the newest of the minor,
# installed into the tool cache by its own setup.sh, run by bash as setup-python runs it.
RUN . /etc/os-release; \
    url=$(curl -fsSL --retry 5 https://raw.githubusercontent.com/actions/python-versions/main/versions-manifest.json \
      | python3 -c 'import json, sys; m, v = sys.argv[1:]; print(next(f["download_url"] for r in json.load(sys.stdin) if r["stable"] and r["version"].startswith(m + ".") for f in r["files"] if (f["platform"], f.get("platform_version"), f["arch"]) == ("linux", v, "x64")))' \
        "${PYTHON}" "${VERSION_ID}"); \
    mkdir /tmp/python; \
    curl -fsSL --retry 5 "${url}" | tar -xz -C /tmp/python; \
    (cd /tmp/python && bash ./setup.sh > /dev/null); \
    rm -rf /tmp/python

# Its environment as the inputs action (.github/actions/inputs) makes it: a virtual environment of the
# requirements and what the package's build asks; the package itself is installed at each run.
COPY --chown=${USER} requirements.txt pyproject.toml /tmp/inputs/
RUN python=$(ls -d "${RUNNER_TOOL_CACHE}/Python/${PYTHON}".*/x64/bin/python3 | sort -V | tail -1); \
    "${python}" -m venv "/home/${USER}/.ci-venv"; \
    build=$("/home/${USER}/.ci-venv/bin/python" -c "import sys, tomllib; print(' '.join(tomllib.load(open(sys.argv[1], 'rb'))['build-system']['requires']))" /tmp/inputs/pyproject.toml); \
    "/home/${USER}/.ci-venv/bin/python" -m pip install -q --retries 10 --timeout 60 -r /tmp/inputs/requirements.txt ${build}
ENV PATH=/home/${USER}/.ci-venv/bin:${PATH} VIRTUAL_ENV=/home/${USER}/.ci-venv CI=true
WORKDIR /home/${USER}
