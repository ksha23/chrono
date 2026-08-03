#!/usr/bin/env python3
"""Single entry point for the Chrono::Sensor Metal RT test suite.

    python3 docs/showcase/tools/run_tests.py            # run everything available locally
    python3 docs/showcase/tools/run_tests.py --build     # build first, then run
    python3 docs/showcase/tools/run_tests.py --only t1   # one tier
    python3 docs/showcase/tools/run_tests.py --list      # show what would run

Runs every tier that does not need a second GPU:

  tier 0  golden-image regression      docs/showcase/tools/golden.py
  tier 1  analytic ground truth        docs/showcase/demos/bin/verify_render_math
  tier 2  statistical / convergence    build/bin/utest_SEN_metal_stochastic
  tier 3  backend-agnostic unit tests  build/bin/utest_SEN_{gps,data_access,interface,threadsafety,radar}
  dyn     dynamic sensors              docs/showcase/demos/bin/verify_dynamic_sensors

Tier 4 (cross-backend parity vs OptiX) is deliberately NOT here: it needs an NVIDIA GPU
and it is a report rather than a gate. See docs/showcase/tools/PARITY.md.

GoogleTest binaries are enumerated and each case is run in its OWN PROCESS. That costs a
few hundred milliseconds and buys crash isolation: one segfaulting case cannot hide the
results of the cases that would have run after it. There is currently exactly one such
case (see KNOWN below), which is precisely why this matters.

Exit code is non-zero if anything fails, including the already-diagnosed backend bugs
listed in KNOWN. Those are reported separately from new failures so a regression is never
buried in the noise of an open bug; pass --ignore-known to exit 0 when the known bugs are
the only thing red.
"""

import argparse
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
BUILD = os.environ.get("CHRONO_BUILD", os.path.join(REPO, "build"))
DEMO_BIN = os.path.join(REPO, "docs", "showcase", "demos", "bin")

# Failures we have already diagnosed and reported. Keeping them here, with the reason and
# where the diagnosis lives, means a NEW failure is instantly visible instead of being lost
# in a wall of expected red. Remove an entry the moment the underlying bug is fixed.
KNOWN = {
    "t1/verify_render_math":
        "ChDepthCamera/ChNormalCamera/ChSegmentationCamera hFOV is ignored: "
        "ChFilterMetalRTRender.mm only reads GetHFOV() through a dynamic_pointer_cast to "
        "ChCameraSensor, which none of the three derive from, so all three render at the "
        "hard-coded 1.408 rad fallback. verify_render_math measures the effective hFOV in "
        "closed form and prints the one-line fix.",
    "t3/utest_SEN_interface::SensorInterface.shapes":
        "SEGFAULT. ChSensorManager::ReconstructScenes() reaches ChMetalRTEngine, but "
        "ChFilterMetalRTRender latches m_renderer_built and only ever calls UpdateDynamic() "
        "afterwards, which indexes buffers sized at build time. Adding geometry at runtime "
        "therefore walks off the end. Runtime add/remove of bodies is unsupported on Metal.",
}

GTESTS = {
    "t2": ["utest_SEN_metal_stochastic"],
    "t3": ["utest_SEN_gps", "utest_SEN_data_access", "utest_SEN_interface",
           "utest_SEN_threadsafety", "utest_SEN_radar"],
}

RESET, BOLD, RED, GREEN, YELLOW, DIM = "\033[0m", "\033[1m", "\033[31m", "\033[32m", "\033[33m", "\033[2m"


def color(s, c):
    return s if not sys.stdout.isatty() else f"{c}{s}{RESET}"


class Result:
    def __init__(self, tier, name, ok, secs, detail="", crashed=False):
        self.tier, self.name, self.ok, self.secs = tier, name, ok, secs
        self.detail, self.crashed = detail, crashed

    @property
    def key(self):
        return f"{self.tier}/{self.name}"

    @property
    def known(self):
        return KNOWN.get(self.key)


def run(cmd, cwd=REPO, capture=True):
    t0 = time.time()
    p = subprocess.run(cmd, cwd=cwd, capture_output=capture, text=True)
    return p, time.time() - t0


def gtest_cases(binary):
    """Enumerate Suite.Case names so each can be run in its own process."""
    p, _ = run([binary, "--gtest_list_tests"])
    if p.returncode != 0:
        return []
    cases, suite = [], None
    for line in p.stdout.splitlines():
        if not line.strip() or line.startswith("Running main"):
            continue
        if not line.startswith(" "):
            suite = line.strip().rstrip(".")
        elif suite:
            case = line.strip().split("#")[0].strip()
            if case:
                cases.append(f"{suite}.{case}")
    return cases


def run_gtest_binary(tier, name, results, verbose):
    path = os.path.join(BUILD, "bin", name)
    if not os.path.exists(path):
        results.append(Result(tier, name, False, 0.0, f"not built: {path}"))
        return
    cases = gtest_cases(path)
    if not cases:
        results.append(Result(tier, name, False, 0.0, "could not enumerate test cases"))
        return
    for case in cases:
        p, secs = run([path, f"--gtest_filter={case}"])
        crashed = p.returncode < 0 or p.returncode > 1
        ok = p.returncode == 0
        detail = ""
        if crashed:
            sig = -p.returncode if p.returncode < 0 else p.returncode - 128
            detail = f"crashed (signal {sig})"
        elif not ok:
            fails = [l.strip() for l in p.stdout.splitlines() if "Failure" in l or l.strip().startswith("Expected")]
            detail = fails[0][:110] if fails else "assertion failed"
        if verbose:
            sys.stdout.write(p.stdout)
        results.append(Result(tier, f"{name}::{case}", ok, secs, detail, crashed))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--build", action="store_true", help="build the library, unit tests and demos first")
    ap.add_argument("--only", action="append", default=None, metavar="TIER",
                    help="run only these tiers (t0, t1, t2, t3, dyn); repeatable")
    ap.add_argument("--list", action="store_true", help="list the tiers and exit")
    ap.add_argument("--verbose", "-v", action="store_true", help="stream each test's own output")
    ap.add_argument("--ignore-known", action="store_true",
                    help="exit 0 when the only failures are the already-diagnosed bugs in KNOWN")
    args = ap.parse_args()

    tiers = ["t0", "t1", "t2", "t3", "dyn"]
    if args.list:
        print("t0   golden-image regression   docs/showcase/tools/golden.py")
        print("t1   analytic ground truth     verify_render_math")
        print("t2   statistical / convergence utest_SEN_metal_stochastic")
        print("t3   backend-agnostic unit tests " + ", ".join(GTESTS["t3"]))
        print("dyn  dynamic sensors           verify_dynamic_sensors")
        print("\n(tier 4, cross-backend parity, needs an NVIDIA GPU: see PARITY.md)")
        return 0
    if args.only:
        tiers = [t for t in tiers if t in args.only]
        if not tiers:
            raise SystemExit(f"run_tests: nothing matches --only {args.only}")

    if args.build:
        print(color("== building ==", BOLD))
        targets = ["Chrono_sensor"] + GTESTS["t2"] + GTESTS["t3"]
        p, secs = run(["ninja", "-C", BUILD] + targets, capture=False)
        if p.returncode != 0:
            raise SystemExit("run_tests: build failed")
        p, secs = run(["bash", os.path.join(REPO, "docs", "showcase", "demos", "build.sh"),
                       "verify_render_math", "verify_golden", "verify_dynamic_sensors"], capture=False)
        if p.returncode != 0:
            raise SystemExit("run_tests: demo build failed")
        print()

    results = []
    t_start = time.time()

    if "t0" in tiers:
        print(color("== tier 0: golden-image regression ==", BOLD))
        p, secs = run([sys.executable, os.path.join(HERE, "golden.py"), "--quiet"])
        if args.verbose or p.returncode != 0:
            sys.stdout.write(p.stdout)
        else:
            tail = (p.stdout.strip().splitlines() or [""])[-1]
            print(color(tail, DIM))
        results.append(Result("t0", "golden_images", p.returncode == 0, secs,
                              "" if p.returncode == 0 else "pixel diff exceeded tolerance"))

    if "t1" in tiers:
        print(color("\n== tier 1: analytic ground truth ==", BOLD))
        binp = os.path.join(DEMO_BIN, "verify_render_math")
        if not os.path.exists(binp):
            results.append(Result("t1", "verify_render_math", False, 0.0, f"not built: {binp}"))
        else:
            p, secs = run([binp])
            sys.stdout.write(p.stdout)
            results.append(Result("t1", "verify_render_math", p.returncode == 0, secs,
                                  "" if p.returncode == 0 else "one or more analytic assertions failed"))

    for tier in ("t2", "t3"):
        if tier not in tiers:
            continue
        label = "statistical / convergence" if tier == "t2" else "backend-agnostic unit tests"
        print(color(f"\n== tier {tier[1]}: {label} ==", BOLD))
        for name in GTESTS[tier]:
            before = len(results)
            run_gtest_binary(tier, name, results, args.verbose)
            new = results[before:]
            bad = [r for r in new if not r.ok]
            tag = color("ok", GREEN) if not bad else color(f"{len(bad)} failing", RED)
            print(f"  {name:<28} {len(new):>2} case(s), {tag}")
            for r in bad:
                mark = "KNOWN" if r.known else ("CRASH" if r.crashed else "FAIL")
                print(f"      [{color(mark, YELLOW if r.known else RED)}] "
                      f"{r.name.split('::', 1)[-1]}  {r.detail}")

    if "dyn" in tiers:
        print(color("\n== dynamic sensors ==", BOLD))
        binp = os.path.join(DEMO_BIN, "verify_dynamic_sensors")
        if not os.path.exists(binp):
            results.append(Result("dyn", "verify_dynamic_sensors", False, 0.0, f"not built: {binp}"))
        else:
            p, secs = run([binp])
            if args.verbose:
                sys.stdout.write(p.stdout)
            last = [l for l in p.stdout.strip().splitlines() if l.startswith("RESULT") or "buffers delivered" in l]
            for l in last:
                print("  " + l)
            results.append(Result("dyn", "verify_dynamic_sensors", p.returncode == 0, secs,
                                  "" if p.returncode == 0 else "a sensor delivered no data"))

    # ------------------------------------------------------------------ summary
    total = len(results)
    passed = [r for r in results if r.ok]
    failed = [r for r in results if not r.ok]
    known = [r for r in failed if r.known]
    new = [r for r in failed if not r.known]

    print(color("\n" + "=" * 78, BOLD))
    print(color("SUMMARY", BOLD))
    print(color("=" * 78, BOLD))
    for tier in tiers:
        rs = [r for r in results if r.tier == tier]
        if not rs:
            continue
        ok = len([r for r in rs if r.ok])
        secs = sum(r.secs for r in rs)
        state = color("PASS", GREEN) if ok == len(rs) else color("FAIL", RED)
        print(f"  {state}  tier {tier:<4} {ok}/{len(rs)} passed   ({secs:.1f}s)")
    print(f"\n  {len(passed)}/{total} checks passed in {time.time() - t_start:.1f}s")

    if known:
        print(color(f"\n  {len(known)} failure(s) are ALREADY-DIAGNOSED BACKEND BUGS:", YELLOW))
        for r in known:
            print(f"    - {r.key}")
            for line in _wrap(r.known, 70):
                print(f"        {line}")
    if new:
        print(color(f"\n  {len(new)} NEW failure(s) -- these are not on the known list:", RED))
        for r in new:
            print(f"    - {r.key}  {r.detail}")

    if not failed:
        print(color("\n  RESULT: ALL CHECKS PASSED", GREEN))
        return 0
    if new:
        print(color("\n  RESULT: FAILURES (including new ones)", RED))
        return 1
    if args.ignore_known:
        print(color("\n  RESULT: only known bugs failed, and --ignore-known was given", YELLOW))
        return 0
    print(color("\n  RESULT: FAILURES (all on the known list; pass --ignore-known to exit 0)", YELLOW))
    return 1


def _wrap(text, width):
    words, line, out = text.split(), "", []
    for w in words:
        if len(line) + len(w) + 1 > width:
            out.append(line)
            line = w
        else:
            line = (line + " " + w).strip()
    if line:
        out.append(line)
    return out


if __name__ == "__main__":
    sys.exit(main())
