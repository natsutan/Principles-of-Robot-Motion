#線分の交差判定を行う関数
import math
import numpy as np


# 点p1, 点p2を通る直線上にある点pが、線分ABに含まれているかどうかを判定する
def is_in_segment(p, p1, p2):
    x = p[0]
    y = p[1]
    x1 = p1[0]
    y1 = p1[1]
    x2 = p2[0]
    y2 = p2[1]

    # 線分1のx方向とy方向のどちらが長いか判定する
    if math.fabs(x1 - x2) > math.fabs(y1 - y2):
        use_x = True
    else:
        use_x = False

    # 長い方を使って、交点が線分の範囲内にあるかを判定
    if use_x:
        min_v = min(x1, x2)
        max_v = max(x1, x2)
        v = x
    else:
        min_v = min(y1, y2)
        max_v = max(y1, y2)
        v = y

    if min_v <= v <= max_v:
        return True
    else:
        return None

    return True

def intersection_det(p1, p2, p3, p4):
    """
    線分の交差判定を行う関数
    :param p1: 線分1の始点
    :param p2: 線分1の終点
    :param p3: 線分2の始点
    :param p4: 線分2の終点
    :return: 交差しているときは交点の座標を返す。交差しないときは、Noneを返す。

    """
    x1 = p1[0]
    y1 = p1[1]
    x2 = p2[0]
    y2 = p2[1]
    x3 = p3[0]
    y3 = p3[1]
    x4 = p4[0]
    y4 = p4[1]

    # 交点の計算
    xi = np.linalg.det(np.array([[x1, y1], [x2, y2]]))
    eta = np.linalg.det(np.array([[x3, y3], [x4, y4]]))
    delta = np.linalg.det(np.array([[y2 - y1, x2 - x1], [y4 - y3, x4 - x3]]))

    x = ((x4 - x3) * xi - (x2 - x1) * eta) / delta
    y = ((y4 - y3) * xi - (y2 - y1) * eta) / delta

    if is_in_segment((x, y), p1, p2) and is_in_segment((x, y), p3, p4):
        return (x, y)
    else:
        return None


def main():
    p1 = (0, 0)
    p2 = (10, 10)
    p3 = (0, 10)
    p4 = (10, 0)

    # 5,5
    print(intersection_det(p1, p2, p3, p4))

    # None
    p1 = (0, 0)
    p2 = (10, 10)
    p3 = (0, 10)
    p4 = (1, 9)
    print(intersection_det(p1, p2, p3, p4))


if __name__ == '__main__':
    main()
