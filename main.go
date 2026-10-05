package main

import (
	"fmt"
	"math"
	"os"
)

// 场景：静止起步 → 爬行 walk 加速 → t=4s 请求切换小跑 trot
// （仿真器会等到当前摆动腿完成摆动、四足均支撑的安全时刻才真正切换）
// → 小跑加速到 0.35 m/s。全程记录帧并导出单文件 HTML 回放。
func main() {
	params := SimParams{
		HipX: 0.20, HipY: 0.12, BodyH: 0.32,
		StepH: 0.05, MinMargin: 0.02,
	}
	const maxStride = 0.45 // 单腿工作空间允许的最大步幅 (m)

	walk := WalkGait(1.0)
	trot := TrotGait(1.6)

	// walk 阶段叠加横向重心摆动（lateral sway）：三足支撑时支撑三角形
	// 的对角边穿过机体中心附近，必须把重心投影摆向支撑侧才能保持正裕度。
	// 摆动顺序 RL→FL→RR→FR：{RL,FL} 摆动时（t mod 1 ∈ [0,0.5)）重心需在
	// 负侧，{RR,FR} 摆动时在正侧——即过零点与换腿时刻对齐的方波，用 tanh
	// 整形正弦近似（快速过零，缩短低裕度区间）：
	//   y_com = -A·tanh(k·sin(2πf·t))
	// 注：β=0.75 的 walk 不存在四足同时支撑区间，换腿瞬间裕度必然过零，
	// 快速过零是让低裕度帧占比最小的标准做法。
	const swayA, swayK = 0.085, 2.5
	swayArg := func(t float64) float64 { return 2 * math.Pi * walk.Freq * t }
	var sim *Sim
	velFn := func(t float64) Vec2 {
		var vx float64
		switch {
		case t < 1.0:
			vx = 0
		case t < 2.0:
			vx = 0.2 * (t - 1.0)
		case t < 4.0:
			vx = 0.2
		case t < 6.0:
			vx = 0.2 + 0.15*(t-4.0)/2.0
		default:
			vx = 0.35
		}
		var vy float64
		if t < 4.5 {
			sn, cs := math.Sin(swayArg(t)), math.Cos(swayArg(t))
			th := math.Tanh(swayK * sn)
			vy = -swayA * swayK * (1 - th*th) * cs * 2 * math.Pi * walk.Freq
		} else if sim != nil {
			vy = -sim.Body.Y // 切换到 trot 后重心缓慢回中
		}
		return Vec2{X: vx, Y: vy}
	}

	// 步态参数预检：在生成轨迹前识别几何上必然不稳定的组合。
	fmt.Println("== 步态参数预检 ==")
	for _, a := range []GaitAnalysis{
		AnalyzeGait(walk, 0.2, maxStride),
		AnalyzeGait(trot, 0.35, maxStride),
		AnalyzeGait(Gait{Name: "walk-β0.6(坏)", Freq: 1.0, Duty: 0.6, Offsets: walk.Offsets}, 0.2, maxStride),
		AnalyzeGait(Gait{Name: "walk-高频低β(坏)", Freq: 3.0, Duty: 0.45, Offsets: walk.Offsets}, 0.2, maxStride),
	} {
		status := "静态稳定 ✓"
		if !a.StaticallyStable {
			status = "静态不稳定 ✗"
		}
		fmt.Printf("  %-18s 最少支撑足=%d 步幅=%.3fm → %s\n", a.Gait.Name, a.MinStanceLegs, a.Stride, status)
		for _, iss := range a.Issues {
			fmt.Printf("      ⚠ %s\n", iss)
		}
	}

	sim = NewSim(params, walk, velFn)
	const dt = 0.005
	const duration = 12.0
	const switchReqT = 4.0

	var frames []Frame
	switchRequested := false
	for t := 0.0; t < duration; t += dt {
		if !switchRequested && t >= switchReqT {
			sim.RequestSwitch(trot)
			switchRequested = true
			fmt.Printf("\n== t=%.2fs 请求切换 walk → trot（等待安全切换点…）\n", t)
		}
		f := sim.Step(dt)
		frames = append(frames, f)
	}

	for _, sw := range sim.Switches {
		fmt.Printf("== t=%.2fs 实际切换：%s → %s（四足均已触地，轨迹连续）\n", sw.T, sw.From, sw.To)
	}

	// 统计稳定性（按实际切换时刻分段）
	unstableWalk, unstableTrot, nWalk := 0, 0, 0
	switchT := math.Inf(1)
	if len(sim.Switches) > 0 {
		switchT = sim.Switches[0].T
	}
	for _, f := range frames {
		if f.T < switchT {
			nWalk++
			if !f.Stable {
				unstableWalk++
			}
		} else if !f.Stable {
			unstableTrot++
		}
	}
	fmt.Printf("\n== 回放统计 ==\n  总帧数 %d\n  walk 段：%d 帧，裕度不足 %d 帧\n  trot 段：%d 帧，裕度不足 %d 帧（trot 为动态步态，双足支撑区间被判静态不稳定属预期）\n",
		len(frames), nWalk, unstableWalk, len(frames)-nWalk, unstableTrot)

	sanitizeFrames(frames)
	notes := []string{
		"相位模型: φ_i = frac(f·t + o_i)，φ<β 为支撑相，否则为摆动相",
		"摆线轨迹: 起止速度为零，落地无速度跳变",
		"静态稳定判据: 重心投影须落在支撑多边形内且裕度 ≥ 阈值",
	}
	out := "replay.html"
	if err := WriteHTML(out, PageData{Frames: frames, Switches: sim.Switches, Params: params, Notes: notes}); err != nil {
		fmt.Fprintln(os.Stderr, "写出回放文件失败:", err)
		os.Exit(1)
	}
	info, _ := os.Stat(out)
	fmt.Printf("== 已生成 %s（%.1f KB），用浏览器打开即可回放\n", out, float64(info.Size())/1024)

	//  sanity：全程支撑足位置不得跳变
	maxJump := 0.0
	for k := 1; k < len(frames); k++ {
		for i := 0; i < 4; i++ {
			if frames[k].Stance[i] && frames[k-1].Stance[i] {
				d := math.Hypot(frames[k].Feet[i].X-frames[k-1].Feet[i].X, frames[k].Feet[i].Y-frames[k-1].Feet[i].Y)
				if d > maxJump {
					maxJump = d
				}
			}
		}
	}
	fmt.Printf("== 连续性自检：相邻帧支撑足最大位移 %.2e m（应为 0）\n", maxJump)
}
