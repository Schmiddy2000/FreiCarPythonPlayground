# Steps A to C: numbers

| | outer | inner1 | inner2 | inner3 | inner4 | inner5 | inner6 | inner7 | inner8 | inner9 |
|---|---|---|---|---|---|---|---|---|---|---|
| raw points | 68388 | 15221 | 6186 | 6084 | 7451 | 8030 | 7980 | 22020 | 10120 | 21526 |
| hampel replaced | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| hampel max mm | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 |
| gate dropped | 57369 | 13106 | 5364 | 5316 | 6418 | 6857 | 6829 | 18627 | 8649 | 18347 |
| gate max mm | 2.42 | 8.24 | 4.98 | 2.24 | 1.51 | 2.42 | 2.84 | 1.53 | 9.54 | 6.02 |
| loops erased | 7 | 7 | 7 | 1 | 0 | 3 | 0 | 1 | 4 | 2 |
| loop points removed | 15 | 15 | 14 | 2 | 0 | 6 | 0 | 2 | 10 | 5 |
| loop length cm | 8.39 | 8.35 | 7.50 | 1.01 | 0 | 3.07 | 0 | 1.09 | 6.13 | 3.03 |
| loop max mm | 31.60 | 38.02 | 28.32 | 10.06 | 0.00 | 9.57 | 0.00 | 0.53 | 20.24 | 14.04 |
| overlap points removed | 33 | 23 | 26 | 19 | 37 | 0 | 21 | 27 | 38 | 47 |
| overlap length cm | 34.99 | 23.64 | 27.80 | 19.07 | 39.32 | 0.00 | 22.37 | 28.47 | 40.43 | 50.56 |
| overlap max mm | 1.26 | 3.43 | 1.44 | 2.25 | 3.21 | 0.00 | 2.67 | 1.95 | 3.41 | 1.84 |
| closure gap mm | 1.31 | 3.92 | 3.72 | 1.32 | 3.30 | 0.51 | 3.78 | 0.91 | 6.07 | 5.47 |
| corners | 2 | 6 | 4 | 2 | 0 | 4 | 3 | 1 | 4 | 6 |
| smooth mean mm | 0.33 | 0.44 | 0.40 | 0.67 | 0.61 | 0.39 | 0.47 | 0.38 | 0.44 | 0.41 |
| smooth max mm | 13.50 | 8.23 | 5.88 | 12.89 | 3.28 | 10.43 | 14.02 | 5.59 | 10.36 | 7.48 |
| final points | 10971 | 2077 | 782 | 747 | 996 | 1167 | 1130 | 3364 | 1423 | 3127 |
| final perimeter m | 120.128 | 22.642 | 8.627 | 8.086 | 10.791 | 12.860 | 12.279 | 36.755 | 15.507 | 33.962 |

## Raster

- grid: 1396 rows x 2827 cols, 1 cm cells, origin (-8.12, -5.38)
- exterior: outer, islands: inner1, inner2, inner3, inner4, inner5, inner6, inner7, inner8, inner9
- road area: 136.221 m2 (polygon 136.214 m2, error +0.00%)

## Ground truth (simulated recordings)

- road mask vs true mask: IoU 0.9981, 1320 extra and 1267 missing cells

| boundary | mean mm | p95 mm | max mm | operator back-ups | operator overshoots | loops erased |
|---|---|---|---|---|---|---|
| outer | 0.83 | 1.99 | 15.51 | 3 | 1 | 7 |
| inner1 | 0.89 | 2.11 | 8.68 | 1 | 3 | 7 |
| inner2 | 0.88 | 2.53 | 6.92 | 1 | 3 | 7 |
| inner3 | 1.13 | 2.24 | 14.35 | 0 | 0 | 1 |
| inner4 | 1.04 | 2.90 | 4.46 | 0 | 0 | 0 |
| inner5 | 0.91 | 2.40 | 11.12 | 1 | 1 | 3 |
| inner6 | 1.06 | 2.04 | 14.78 | 0 | 0 | 0 |
| inner7 | 0.84 | 1.99 | 5.69 | 1 | 0 | 1 |
| inner8 | 0.84 | 1.94 | 11.60 | 0 | 3 | 4 |
| inner9 | 0.86 | 2.20 | 8.71 | 1 | 2 | 2 |
