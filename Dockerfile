# Reproducibility image for the multi-feed transit-routing study.
#
#   docker build -t namma-metro-router .
#   docker run --rm -v "$PWD/results:/work/results" namma-metro-router
#
# WHAT THIS IS FOR
# ════════════════
# Every figure this project publishes comes out of one command inside this
# image. That is worth more than it sounds: SEA and ALENEX run artifact
# evaluation tracks, and a reviewer who can reproduce an experiment does not
# have to take the author's word for anything — including, in this case, who
# wrote which line of the implementation. In an empirical paper the contribution
# is the experimental design and the finding, and both are checkable here.
#
# WHAT IT CANNOT DO, STATED UP FRONT
# ══════════════════════════════════
# It cannot reproduce the exact INPUTS. Transit agencies republish their feeds
# continuously, at the same URL, with no version and no archive — this project
# has already been bitten by that once, when BART added roughly 40% more service
# between two runs and every published figure moved. So:
#
#   * feeds/feeds.lock.json records the SHA-256 of every feed the numbers came
#     from, so you can always tell whether you are looking at the same input;
#   * `scripts/fetch_feeds.py --verify` checks a local copy against those pins;
#   * a run against different bytes still reproduces the METHOD exactly, and
#     says so, rather than quietly producing different numbers.
#
# It also cannot reproduce the exact LATENCIES: those depend on the host CPU,
# its thermal state and whether the machine is on mains power (measured on this
# project's own hardware: p50 roughly doubles on battery). The structural and
# frequency results — the trade-off rates, the topology metrics, the optimality
# gaps — are deterministic given the same feeds, and those are the results the
# study's conclusions rest on.

FROM ubuntu:24.04

# Pinned to the distribution's toolchain rather than a rolling one, so the same
# image tag keeps building the same compiler.
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        ca-certificates \
        cmake \
        curl \
        git \
        ninja-build \
        python3 \
        unzip \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /work

# Sources first, then the build, so an edit to a script does not invalidate the
# compiled layer.
COPY CMakeLists.txt LICENSE ./
COPY include/ include/
COPY src/ src/
COPY tests/ tests/
COPY tools/ tools/

# googletest is fetched at configure time. Doing it during the build means the
# image is self-contained afterwards: the reproduce step needs the network only
# for the transit feeds themselves.
RUN cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -G Ninja \
    && cmake --build build -j"$(nproc)" \
    && cmake -S . -B build_debug -DCMAKE_BUILD_TYPE=Debug -G Ninja \
    && cmake --build build_debug -j"$(nproc)"

COPY scripts/ scripts/
COPY docs/ docs/
COPY README.md ./

# Mount a host directory here to keep the outputs:
#   docker run --rm -v "$PWD/results:/work/results" namma-metro-router
VOLUME ["/work/results", "/work/feeds"]

ENTRYPOINT ["bash", "scripts/reproduce.sh"]
CMD []
