# Hotel stairs benchmark

Default go2-scan multi-floor scene: three complete floors, two static stair
groups, rooms/corridors/furniture, and sealed elevator shafts. Normal interior
doors were removed after visual review; the L1 lobby outer door remains closed.
Elevators are forbidden, not controlled simulation objects.

Use `scene:=hotel_stairs` explicitly, or request `multi_floor:=true` without a
scene to select it. Depot remains `scene:=depot` for regression only.
