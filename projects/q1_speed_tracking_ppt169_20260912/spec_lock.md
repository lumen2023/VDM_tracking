# Execution Lock

## canvas
- viewBox: 0 0 1280 720
- format: PPT 16:9
- margin: left 80, right 80, top 56, bottom 48
- content_area: x 80..1200, y 140..660
- header_y: 56
- footer_y: 664

## colors
- bg: #FFFFFF
- bg_secondary: #F2F2F2
- primary: #17375E
- accent: #FFC000
- secondary_accent: #002060
- text: #404040
- text_secondary: #757070
- text_tertiary: #A6A6A6
- border: #D9D9D9
- success: #2E7D32
- warning: #E92E25

## typography
- font_family: "Microsoft YaHei", "PingFang SC", Arial, sans-serif
- title_family: "Century Gothic", "Microsoft YaHei", "PingFang SC", sans-serif
- emphasis_family: "Century Gothic", Arial, sans-serif
- code_family: Consolas, "Courier New", monospace
- body: 20
- cover_title: 64
- title: 34
- hero_number: 36
- subtitle: 26
- annotation: 15
- footnote: 12

## icons
- library: tabler-outline
- stroke_width: 2
- inventory: ruler-measure, gauge, route-2, math-function, trending-up, alert-triangle, circle-x, target, bulb, circle-check

## images
- fig01_steer_vs_v: images/fig01_steer_vs_v.png | no-crop
- fig03_an_vs_v: images/fig03_an_vs_v.png | no-crop
- fig06_trajectory_pp_low: images/fig06_trajectory_pp_low.png | no-crop
- fig11_cut_regression: images/fig11_cut_regression.png | no-crop
- fig04_lateral_max_vs_v: images/fig04_lateral_max_vs_v.png | no-crop
- fig17_pp_high_lateral_error: images/fig17_pp_high_lateral_error.png | no-crop
- fig13_steer_raw_vs_limited: images/fig13_steer_raw_vs_limited.png | no-crop

## page_rhythm
- P01: anchor
- P02: dense
- P03: dense
- P04: dense
- P05: dense
- P06: dense
- P07: breathing
- P08: anchor

## page_layouts
- layout_dir: templates/dongda/svg-flat
- P01: slide_01
- P08: slide_25

## page_charts
- P02: vertical_list
- P03: basic_table
- P05: dumbbell_chart
- P06: pros_cons_chart
- P07: vertical_list

## forbidden
- Mixing icon libraries
- rgba()
- `<style>`, `class`, `<foreignObject>`, `textPath`, `@font-face`, `<animate*>`, `<script>`, `<iframe>`, `<symbol>`+`<use>`
- `<g opacity>` (set opacity on each child element individually)
- HTML named entities in text — write as raw Unicode; escape `& < > " '` as `&amp; &lt; &gt; &quot; &apos;`
- Reading / opening image files to inspect them — image facts come from this file's `images` section only
- Redrawing with a chart template the data curves already present in an embedded PNG
