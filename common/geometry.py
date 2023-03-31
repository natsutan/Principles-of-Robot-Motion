import math

EPSIRON = 0.01

# 線分の始点と終点、点の座標を与えると、点が線分上にあるかどうかを判定する。
# 交わったときはTrue、交わらないときはFalseを返す。
def is_crossed_line_segment(start, end, p, e=EPSIRON):

    # 線分->点のベクトルがなす角度と、線分の始点->終点のベクトルがなす角度が等しい
    vs = (p[0] - start[0], p[1] - start[1])
    ve = (end[0] - start[0], end[1] - start[1])
    if math.fabs(math.atan2(vs[1], vs[0]) - math.atan2(ve[1], ve[0])) > e:
        return False
    # ベクトルの大きさが、vs <= ve
    if math.sqrt(vs[0] ** 2 + vs[1] ** 2) > math.sqrt(ve[0] ** 2 + ve[1] ** 2):
        return False

    return True

