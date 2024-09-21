import math
import numerical_data as nd
import used_classes as uc
import code_generator as cg
import matplotlib.pyplot as plt


def split_point_list(points_list: list[uc.Point]):
    newlist_x = []
    newlist_y = []
    for i in points_list:
        newlist_x.append(i[1].x)
        newlist_y.append(i[1].y)
    return [newlist_x, newlist_y]


def add_lists(list: list):
    newlist = []
    for s in list:
        for i in s:
            newlist.append(i)
    return newlist


def draw_line(points):
    for i in range(1, len(points)):
        plt.plot([points[i - 1].x, points[i].x], [points[i - 1].y, points[i].y], "k-")


def draw_line_red(points):
    for i in range(1, len(points)):
        plt.plot([points[i - 1].x, points[i].x], [points[i - 1].y, points[i].y], "r-")


def draw_points(points):
    for i in points:
        plt.plot(i.x, i.y, "o")


def draw_axis(xaxis, yaxis):
    plt.axis([xaxis[0], xaxis[1], yaxis[0], yaxis[1]])


def draw_segment(points):
    draw_points(points)
    draw_line(points)


def results():
    plt.grid()
    plt.show()


def test_anglesearch_speaker():
    for info in cg._CHAMPS_TABLE_SPEAKER:
        gpos = uc.Point.from_tuple(nd._SPEAKER_POINT_1)
        rpos = uc.Point(gpos.x - info.distance, 0)

        model = uc.Model(rpos, gpos, info.rpm)
        pm = uc.ProjectileMotion(nd._TIME_CHANGE, nd._USE_DRAG)
        vector = uc.Vector(
            uc.angle_search(model, pm, uc.PruneType.NONE, [gpos, uc.Point.from_tuple(nd._SPEAKER_POINT_2)], False),
            model.get_vel(info.rpm),
        )
        exit2 = uc.Vector(vector.angle, nd._SHOOTER_LENGTH).topoint().plus(rpos)
        points = pm.get_points(vector, exit2)

        draw_points([rpos, exit2])
        draw_line(add_lists([[rpos, exit2], points]))
    draw_points([gpos])

    draw_line_red([uc.Point(gpos.x + 0.229997, 0), uc.Point(gpos.x + 0.229997, 1.984502)])
    draw_line_red([uc.Point(gpos.x - 0.923798, 1.984502), uc.Point(gpos.x, 2.492502)])
    draw_axis([-0.1, 10], [-0.1, 10])

    results()


def test_anglesearch_floor():
    gpos = uc.Point.from_tuple(nd._FLOOR_SPOT)
    for info in cg._CHAMPS_TABLE_FLOOR:
        rpos = uc.Point(gpos.x - info.distance, 0)

        model = uc.Model(rpos, gpos, info.rpm)
        pm = uc.ProjectileMotion(nd._TIME_CHANGE, nd._USE_DRAG)
        if info.distance > nd._DO_HEIGHT_DISTANCE:
            do_height = True
        else:
            do_height = False
        vector = uc.Vector(uc.angle_search(model, pm, uc.PruneType.NONE, [gpos], do_height), model.get_vel(info.rpm))
        exit2 = (
            uc.Vector(vector.angle, nd._SHOOTER_LENGTH)
            .topoint()
            .plus(rpos)
            .plus(
                uc.Point(nd._SHOOTER_X_OFFSET_RELATIVE_TO_ROBOT_CENTER, nd._SHOOTER_Y_OFFSET_RELATIVE_TO_ROBOT_CENTER)
            )
        )
        points = pm.get_points(vector, exit2)

        draw_points([rpos, exit2])
        draw_line(add_lists([[rpos, exit2], points]))
    draw_points([gpos])
    draw_axis([-0.1, 10], [-0.1, 10])
    draw_line([uc.Point(nd._STAGE_DIST_FROM_GOAL, 0), uc.Point(nd._STAGE_DIST_FROM_GOAL, nd._STAGE_HEIGHT)])
    results()


# test_anglesearch_speaker()
test_anglesearch_floor()
