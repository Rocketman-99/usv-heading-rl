"""HANDOFF.md 10장 4번: Nomoto 1차 모델 계단 응답 확인.

타각 계단 입력에 대한 선수각속도 응답이 물리적으로 타당한지 확인한다.

    C1  수치적분이 해석해와 일치하는가        r(t) = K*delta*(1 - e^(-t/T))
    C2  정상상태 선수각속도가 K*delta 인가
    C3  63.2% 도달 시간이 T 인가
    C4  부호가 맞는가 (양의 타각 -> 양의 선수각속도 -> 선수각 증가)
    C5  선회 반경이 선체 길이 대비 물리적으로 타당한가
    C6  구동기 포화/변화율 제한이 실제로 지켜지는가

실행::

    python scripts/run_step_response_check.py
    python scripts/run_step_response_check.py --set otter --delta 5

같은 내용이 tests/test_sim.py 에 pytest 로도 들어 있다. 이 스크립트는 사람이
눈으로 보기 위한 것이고, CI 가 막는 것은 pytest 쪽이다.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from usv_heading import (  # noqa: E402
    SimOptions,
    list_param_sets,
    nomoto_params,
    simulate_heading,
)

D2R = math.pi / 180.0
R2D = 180.0 / math.pi

_all_ok = True


def report(name: str, ok: bool, detail: str) -> None:
    global _all_ok
    _all_ok = _all_ok and ok
    print(f"  [{'PASS' if ok else 'FAIL'}] {name:<38} {detail}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--set", dest="set_name", default="mariner",
                    help="파라미터 세트 이름")
    ap.add_argument("--delta", type=float, default=5.0,
                    help="계단 타각 [deg]")
    ap.add_argument("--list", action="store_true", help="파라미터 세트 목록 출력")
    ap.add_argument("--save", default="figures/step_response_{name}.png",
                    help="그림 저장 경로. 빈 문자열이면 저장 안 함")
    args = ap.parse_args()

    if args.list:
        for s in list_param_sets():
            print(f"  {s.name:<10} K = {s.K:+.5f} [{s.k_unit}], "
                  f"T = {s.T:+8.2f} s   ({s.input_type})")
        return 0

    p = nomoto_params(args.set_name)
    delta = args.delta * D2R

    print("=" * 62)
    print(f" Nomoto 계단 응답 확인 — 세트 {p.name!r}")
    print("=" * 62)
    print(f"K = {p.K:+.5f} [{p.k_unit}]")
    print(f"T = {p.T:+.2f} s")
    print(f"입력 종류 : {p.input_type}")
    print(f"출처 : {p.source}")
    if p.notes:
        print(f"주의 : {p.notes}")
    print()

    # ---- 개루프 계단 응답 (구동기 동특성 없음) ----
    p_ol = p.with_actuator(enable=False, delta_max=math.inf)
    out = simulate_heading(p_ol, None, SimOptions(delta_open_loop=delta))
    h = out.opts.h

    r_analytic = p.K * delta * (1 - np.exp(-out.t / p.T))
    rel_err = np.max(np.abs(out.r - r_analytic)) / np.max(np.abs(r_analytic))
    report("C1 수치적분 vs 해석해", rel_err < 1e-6,
           f"최대 상대오차 {rel_err:.2e} (허용 1e-6)")

    r_ss, r_exact = out.r[-1], p.K * delta
    residual = math.exp(-out.opts.t_end / abs(p.T))
    report("C2 정상상태 선수각속도 = K*delta",
           abs(r_ss - r_exact) / abs(r_exact) < 1e-3,
           f"r_ss = {r_ss * R2D:.5f}, K*delta = {r_exact * R2D:.5f} deg/s "
           f"(잔차 e^(-tEnd/T) = {residual:.1e})")

    i63 = int(np.argmax(np.abs(out.r) >= 0.632 * abs(r_exact)))
    report("C3 63.2% 도달 시간 = T", abs(out.t[i63] - abs(p.T)) / abs(p.T) < 0.02,
           f"T63 = {out.t[i63]:.2f} s, T = {p.T:.2f} s")

    sign_ok = np.sign(r_ss) == np.sign(r_exact) and out.psi[-1] * np.sign(r_ss) > 0
    report("C4 부호 정합", sign_ok,
           f"r_ss = {r_ss * R2D:+.4f} deg/s, psi(end) = {out.psi[-1] * R2D:+.1f} deg")

    # ---- C5 선회 반경 ----
    if p.input_type == "rudder_angle" and math.isfinite(p.L) and math.isfinite(p.U):
        R = p.U / abs(r_ss)
        print(f"\n  [C5] delta = {args.delta:g} deg 정상선회:")
        print(f"       선회 반경 R = {R:.1f} m = {R / p.L:.2f} L "
              f"(L = {p.L:.2f} m, U = {p.U:.3f} m/s)")
        print("       선형 Nomoto 는 큰 타각에서 선회 능력을 과대평가한다.")
        print("       비선형 Mariner 모델은 같은 조건에서 6.50 L 이다 (2.2배 차이).")
        print("       docs/02_plant_model.md 의 선형/비선형 비교표 참조.")
    else:
        print("\n  [C5] 선회 반경 확인 건너뜀 (입력이 타각이 아니거나 L, U 미정).")

    # ---- C6 구동기 제한 ----
    if p.actuator.enable:
        big = 2 * p.actuator.delta_max
        act = simulate_heading(
            p, None, SimOptions(t_end=4 * abs(p.T), delta_open_loop=big)
        )
        rate = np.max(np.abs(np.diff(act.delta))) / act.opts.h
        print()
        report("C6a 타각 변화율 제한 준수",
               rate <= p.actuator.delta_rate_max * (1 + 1e-3),
               f"관측 최대 {rate * R2D:.3f} deg/s, "
               f"한계 {p.actuator.delta_rate_max * R2D:.3f} deg/s")
        report("C6b 타각 포화 준수",
               np.max(np.abs(act.delta)) <= p.actuator.delta_max * (1 + 1e-9),
               f"관측 최대 {np.max(np.abs(act.delta)) * R2D:.2f} deg, "
               f"한계 {p.actuator.delta_max * R2D:.2f} deg")

    print("\n" + "=" * 62)
    print(" 전체 통과 — 플랜트 구현이 해석해 및 물리적 타당성과 일치"
          if _all_ok else " 실패 항목 있음 — docs/journal.md 에 기록하고 원인 확인")
    print("=" * 62)

    # ---- 플롯 ----
    if args.save:
        _plot(p, out, r_analytic, args.delta, args.save.format(name=p.name), h)

    return 0 if _all_ok else 1


def _plot(p, out, r_analytic, delta_deg, path, h):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    Path(path).parent.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(2, 1, figsize=(9, 6), sharex=True)

    axes[0].plot(out.t, r_analytic * R2D, "k--", lw=2, label="analytic")
    axes[0].plot(out.t, out.r * R2D, "r-", lw=1.2, label="RK4 numerical")
    axes[0].axvline(abs(p.T), color="b", ls=":", lw=1)
    axes[0].set_ylabel("r [deg/s]")
    axes[0].set_title(
        f"{p.name}: delta = {delta_deg:g} deg step "
        f"(K = {p.K:.4f} 1/s, T = {p.T:.1f} s)"
    )
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc="lower right")

    axes[1].plot(out.t, out.psi * R2D, lw=1.5)
    axes[1].set_ylabel("psi [deg]")
    axes[1].set_xlabel("t [s]")
    axes[1].grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(path, dpi=130)
    print(f"\n그림 저장: {path}")


if __name__ == "__main__":
    raise SystemExit(main())
