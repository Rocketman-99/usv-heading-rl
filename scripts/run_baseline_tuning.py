"""HANDOFF.md 10장 5번: 베이스라인 PID 수동 튜닝 도구.

┌────────────────────────────────────────────────────────────────────┐
│ 이 스크립트는 게인을 "찾아주지 않는다".                              │
│ 작업자(David)가 게인을 직접 바꿔가며 응답과 지표를 본다.             │
│ HANDOFF.md 5장: 베이스라인 PID 게인 튜닝은 대행 금지 항목이다.       │
│ HANDOFF.md 9장: 자동 튜닝 스크립트로 대체하는 것은 금지다.           │
│ 이 파일에 최적화·격자탐색·pidtune 류를 추가하지 말 것.               │
└────────────────────────────────────────────────────────────────────┘

사용법::

    python scripts/run_baseline_tuning.py --kp 3 --ki 0 --kd 200
    python scripts/run_baseline_tuning.py --kp 3 --ki 0 --kd 200 --step 5
    python scripts/run_baseline_tuning.py            # 게인 없이 실행하면 플랜트 정보만

인자 없이 실행하면 플랜트 개루프 특성만 출력하고 종료한다.

튜닝 시 볼 것 (HANDOFF.md 8장 3항):
  - 지표만 보지 말 것. 타각(3번째 subplot)이 지속 진동하면 그 게인은 버린다.
  - 이 플랜트는 이미 적분기를 갖는다(type 1). 계단 목표의 정상상태 오차는
    P 제어만으로도 0 이 된다. Ki 는 상수 외란 제거용이며, 1단계에는 외란이
    없으므로 Ki 를 키우면 오버슈트와 진동만 늘어난다.
    (근거: tests/test_control.py::test_p_control_alone_has_no_steady_state_error)

채택 후 기록할 것:
  docs/03_baseline_pid.md  게인 값, 시험 조건, 지표표, 응답 그림
  docs/journal.md          버린 시도와 그 이유
  docs/decisions.md        D-005 (베이스라인 게인 채택 근거)
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from usv_heading import (  # noqa: E402
    PIDGains,
    PIDHeading,
    SimOptions,
    heading_metrics,
    metrics_table,
    nomoto_params,
    simulate_heading,
)

D2R = math.pi / 180.0
R2D = 180.0 / math.pi


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--set", dest="set_name", default="mariner")
    ap.add_argument("--kp", type=float, default=None, help="비례 게인 [-]")
    ap.add_argument("--ki", type=float, default=None, help="적분 게인 [1/s]")
    ap.add_argument("--kd", type=float, default=None, help="미분 게인 [s]")
    ap.add_argument("--step", type=float, default=10.0, help="목표 침로 계단 [deg]")
    ap.add_argument("--band", type=float, default=0.05, help="정정 판정 대역 [-]")
    ap.add_argument("--ss-frac", type=float, default=0.30,
                    help="정상상태 구간 (뒤쪽 비율)")
    ap.add_argument("--save", default="figures/baseline_pid.png",
                    help="그림 저장 경로. 빈 문자열이면 저장 안 함")
    args = ap.parse_args()

    p = nomoto_params(args.set_name)

    if None in (args.kp, args.ki, args.kd):
        _print_plant_info(p)
        return 0

    gains = PIDGains(Kp=args.kp, Ki=args.ki, Kd=args.kd)

    # 시험 조건. 최종 검증 조건 설계는 작업자 담당이다 (HANDOFF 5장, D-008).
    opts = SimOptions(t_end=6 * abs(p.T), psi_ref=args.step * D2R)
    out = simulate_heading(p, PIDHeading(gains, p), opts)
    m = heading_metrics(out, band=args.band, ss_frac=args.ss_frac)

    print(f"\n게인: {gains}   |   계단 {args.step:g} deg, 플랜트 {p.name!r}")
    print()
    print(metrics_table([m], [str(gains)]))
    print()

    if args.save:
        from usv_heading.plotting import plot_heading_response

        Path(args.save).parent.mkdir(parents=True, exist_ok=True)
        import matplotlib
        matplotlib.use("Agg")
        plot_heading_response(
            out,
            ["baseline PID"],
            f"{p.name}: step {args.step:g} deg, {gains}",
            savepath=args.save,
        )
        print(f"그림 저장: {args.save}")

    print()
    print("타각 이력(3번째 subplot)에 지속 진동이 보이면 지표가 좋아도 채택하지 않는다.")
    print("채택 시 docs/03_baseline_pid.md, journal.md, decisions.md D-005 에 기록할 것.")
    print()
    return 0


def _print_plant_info(p) -> None:
    print()
    print("  게인이 지정되지 않았다. --kp / --ki / --kd 를 주고 다시 실행할 것.")
    print()
    print("  이 플랜트의 개루프 특성:")
    print(f"    K = {p.K:+.5f} 1/s,  T = {p.T:+.2f} s")
    print("    전달함수  psi(s)/delta(s) = K / (s*(1 + T*s))")
    if p.actuator.enable:
        print(f"    타각 한계 {p.actuator.delta_max * R2D:.1f} deg, "
              f"변화율 한계 {p.actuator.delta_rate_max * R2D:.1f} deg/s")
    else:
        print("    구동기 제한 없음")
    print()
    print("  분모에 s 가 있으므로 type 1 이다 — 계단 목표의 정상상태 오차는")
    print("  P 제어만으로 0 이 된다. Ki 는 상수 외란(조류·바람) 제거용이다.")
    print()
    print("  어떤 값부터 시작할지는 작업자가 정한다 (HANDOFF.md 5장 — 대행 금지).")
    print()


if __name__ == "__main__":
    raise SystemExit(main())
