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
- warning_bg: #FDECEA

## typography
- font_family: &quot;Microsoft YaHei&quot;, &quot;PingFang SC&quot;, Arial, sans-serif
- title_family: &quot;Century Gothic&quot;, &quot;Microsoft YaHei&quot;, sans-serif
- emphasis_family: &quot;Century Gothic&quot;, Arial, sans-serif
- code_family: &quot;Consolas&quot;, &quot;Courier New&quot;, monospace
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
- inventory: map-2, route-2, steering-wheel, trending-up, target, alert-triangle, gauge, math-function, git-branch, circle-check, circle-x, bulb

## images
- fig13_chase_cam: images/fig13_chase_cam.gif | no-crop
- fig1_road_graph: images/fig1_road_graph.png | no-crop
- fig3_curvature: images/fig3_curvature.png | no-crop
- fig4_speed_profile: images/fig4_speed_profile.png | no-crop
- fig9_error_vs_speed: images/fig9_error_vs_speed.png | no-crop
- fig11_error_vs_lookahead: images/fig11_error_vs_lookahead.png | no-crop
- fig7_corner_zoom: images/fig7_corner_zoom.png | no-crop

## page_rhythm
- P01: anchor
- P02: breathing
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
- P02: process_flow
- P03: kpi_cards
- P04: bar_chart
- P05: kpi_cards
- P06: basic_table
- P07: vertical_list

## forbidden
- Mixing icon libraries
- rgba()
- `<style>`, `class`, `<foreignObject>`, `textPath`, `@font-face`, `<animate*>`, `<script>`, `<iframe>`, `<symbol>`+`<use>`
- `<g opacity>` (set opacity on each child element individually)
- HTML named entities in text — write as raw Unicode; escape `& < > " '` as `&amp; &lt; &gt; &quot; &apos;`
- Reading / opening image files to inspect them — image facts come from this file's `images` section only
- Redrawing with a chart template the data curves already present in an embedded PNG
- Using any icon outside the `icons.inventory` list above
