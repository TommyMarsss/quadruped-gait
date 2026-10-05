package gait

import (
	"encoding/json"
	"regexp"
	"strings"
	"testing"
)

func TestRenderPayloadSelfContained(t *testing.T) {
	res, err := RunScenario(DefaultGeometry(), DefaultScenario())
	if err != nil {
		t.Fatal(err)
	}
	html, err := RenderPayload(res.Geo, res.LiftHeight, res.Frames,
		res.Touchdowns, res.Switches, ReportMeta{
			Name: "test", Duration: res.Duration, MarginThreshold: MarginThreshold,
		})
	if err != nil {
		t.Fatalf("渲染失败：%v", err)
	}
	s := string(html)

	// 必须是单一完整 HTML 文档，且同时含俯视/侧视两个画布。
	if !strings.HasPrefix(s, "<!DOCTYPE html>") || !strings.Contains(s, "</html>") {
		t.Fatal("输出不是完整 HTML 文档")
	}
	for _, id := range []string{`id="top"`, `id="side"`, `id="payload"`} {
		if !strings.Contains(s, id) {
			t.Errorf("缺少元素 %s", id)
		}
	}

	// 不得引用任何外部资源（http(s) 链接、CDN、外链脚本/样式）。
	for _, pat := range []string{
		`https?://`,
		`<link[^>]+href`,
		`@import`,
		`src\s*=\s*["']https?:`,
	} {
		if regexp.MustCompile(pat).MatchString(s) {
			t.Errorf("报告包含外部依赖，匹配 %s", pat)
		}
	}

	// 内嵌数据必须是合法 JSON，且帧数与仿真一致、字段齐全。
	m := regexp.MustCompile(`(?s)<script type="application/json" id="payload">(.*?)</script>`).FindStringSubmatch(s)
	if m == nil {
		t.Fatal("未找到内嵌 payload")
	}
	var p struct {
		Frames []struct {
			T    float64 `json:"t"`
			Gait string  `json:"gait"`
			Legs []struct {
				X float64 `json:"x"`
				Y float64 `json:"y"`
				Z float64 `json:"z"`
			} `json:"legs"`
			Stable bool     `json:"stable"`
			Margin *float64 `json:"margin"`
		} `json:"frames"`
		Switches []SwitchEvent `json:"switches"`
	}
	if err := json.Unmarshal([]byte(m[1]), &p); err != nil {
		t.Fatalf("内嵌 JSON 非法：%v", err)
	}
	if len(p.Frames) != len(res.Frames) {
		t.Errorf("帧数 %d != %d", len(p.Frames), len(res.Frames))
	}
	if len(p.Switches) != 1 {
		t.Errorf("应有 1 次切换，实际 %d", len(p.Switches))
	}
	// 小跑帧支撑脚为 2 时 margin 必须序列化为 null（而非非法 NaN）。
	var sawNull bool
	for _, f := range p.Frames {
		if f.Gait == "trot" && f.Margin == nil {
			sawNull = true
		}
		if len(f.Legs) != 4 {
			t.Fatalf("每帧必须含 4 条腿，得到 %d", len(f.Legs))
		}
	}
	if !sawNull {
		t.Error("小跑（<3 支撑脚）帧的 margin 应序列化为 null")
	}
}
