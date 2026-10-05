// Command report 运行一段"静止爬行起步 → 爬行加速 → 切换对角小跑"
// 的步态序列，检测落脚速度连续性、步态切换时机与逐帧静态稳定性，
// 并把结果输出为单一离线 HTML 文件 report.html。
package main

import (
	"flag"
	"fmt"
	"math"
	"os"
	"path/filepath"

	"github.com/TommyMarsss/quadruped-gait/gait"
)

func main() {
	out := flag.String("o", "report.html", "输出 HTML 文件路径")
	walkV := flag.Float64("walkv", 0.18, "爬行目标速度 m/s")
	trotV := flag.Float64("trotv", 0.45, "小跑目标速度 m/s")
	walkF := flag.Float64("walkf", 0.8, "爬行步频 Hz")
	trotF := flag.Float64("trotf", 2.0, "小跑步频 Hz")
	switchAt := flag.Float64("switchat", 6.0, "请求步态切换的时刻 s")
	flag.Parse()

	geo := gait.DefaultGeometry()
	cfg := gait.DefaultScenario()
	cfg.WalkV1 = *walkV
	cfg.TrotV = *trotV
	cfg.WalkFreq = *walkF
	cfg.TrotFreq = *trotF
	cfg.SwitchAt = *switchAt

	res, err := gait.RunScenario(geo, cfg)
	if err != nil {
		fmt.Fprintf(os.Stderr, "步态参数不可行：%v\n", err)
		os.Exit(1)
	}

	// ---- 控制台检测摘要 ----
	var maxVz, maxVxy, maxJump float64
	for _, td := range res.Touchdowns {
		maxVz = math.Max(maxVz, math.Abs(td.Vz))
		maxVxy = math.Max(maxVxy, td.VXY)
		maxJump = math.Max(maxJump, td.Jump)
	}
	nUnstable := 0
	minMargin := math.Inf(1)
	for _, f := range res.Frames {
		if !f.Stable {
			nUnstable++
		}
		if !math.IsNaN(f.Margin) {
			minMargin = math.Min(minMargin, f.Margin)
		}
	}
	fmt.Printf("仿真时长 %.2fs，共 %d 帧，落脚事件 %d 次\n",
		res.Duration, len(res.Frames), len(res.Touchdowns))
	fmt.Printf("落脚瞬间最大 |垂直速度| = %.3e m/s（摆线应收敛到 0）\n", maxVz)
	fmt.Printf("落脚瞬间最大 水平速度   = %.3e m/s，位置跳变上界 %.3e m\n", maxVxy, maxJump)
	for _, sw := range res.Switches {
		fmt.Printf("步态切换：%s -> %s，请求于 %.3fs，实际于 %.3fs 执行"+
			"（等待 %.3fs），切换时四足着地=%v，足端位置跳变上界 %.3e m\n",
			sw.From, sw.To, sw.RequestT, sw.SwitchT, sw.SwitchT-sw.RequestT,
			sw.AllGrounded, sw.MaxJump)
	}
	fmt.Printf("静态裕度不足帧 %d/%d；walk 段最小裕度 %.3f m，trot 段按动态步态处理\n",
		nUnstable, len(res.Frames), minMargin)

	html, err := gait.RenderPayload(res.Geo, res.LiftHeight, res.Frames,
		res.Touchdowns, res.Switches, gait.ReportMeta{
			Name:            "静止爬行起步→加速→切换对角小跑",
			Duration:        res.Duration,
			MarginThreshold: gait.MarginThreshold,
		})
	if err != nil {
		fmt.Fprintf(os.Stderr, "渲染报告失败：%v\n", err)
		os.Exit(1)
	}
	if err := os.MkdirAll(filepath.Dir(*out), 0o755); err != nil {
		fmt.Fprintf(os.Stderr, "创建输出目录失败：%v\n", err)
		os.Exit(1)
	}
	if err := os.WriteFile(*out, html, 0o644); err != nil {
		fmt.Fprintf(os.Stderr, "写入报告失败：%v\n", err)
		os.Exit(1)
	}
	fmt.Printf("已生成单文件报告：%s（用浏览器直接打开，无需服务/联网）\n", *out)
}
