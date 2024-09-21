import used_classes as uc
import numerical_data as nd
import math
import dataclasses

_CHAMPS_TABLE_SPEAKER = [
    uc.ShooterInfo(distance=1.38, angle=58.1, rpm=3000.0),  # 0
    uc.ShooterInfo(distance=2.16, angle=47.8, rpm=3000.0),
    uc.ShooterInfo(distance=2.5, angle=42.0, rpm=4000.0),
    uc.ShooterInfo(distance=3.5, angle=33.9635, rpm=4000.0),
    uc.ShooterInfo(distance=4.5, angle=28.20125, rpm=4500.0),
    uc.ShooterInfo(distance=5.5, angle=25.84825, rpm=4800.0),
    uc.ShooterInfo(distance=6.5, angle=21.30525, rpm=4800.0),
    uc.ShooterInfo(distance=7.5, angle=20.27075, rpm=4800.0),
    uc.ShooterInfo(distance=9.0, angle=18.7305, rpm=4800.0),  # 8
]
_CHAMPS_TABLE_FLOOR = [
    # uc.ShooterInfo(distance=0.0, angle=58.1, rpm=1000.0),  # 0
    # uc.ShooterInfo(distance=1.0, angle=47.8, rpm=1000.0),
    # uc.ShooterInfo(distance=1.5, angle=42.0, rpm=1500.0),
    # uc.ShooterInfo(distance=2.0, angle=42.0, rpm=1600.0),
    # uc.ShooterInfo(distance=2.5, angle=42.0, rpm=1700.0),
    # uc.ShooterInfo(distance=3.0, angle=33.9635, rpm=1800.0),
    # uc.ShooterInfo(distance=3.5, angle=33.9635, rpm=1900.0),
    # uc.ShooterInfo(distance=4.0, angle=33.9635, rpm=2000.0),
    # uc.ShooterInfo(distance=4.5, angle=33.9635, rpm=2100.0),
    # uc.ShooterInfo(distance=5.0, angle=33.9635, rpm=2200.0),
    # uc.ShooterInfo(distance=5.5, angle=28.20125, rpm=2400.0),
    # uc.ShooterInfo(distance=6.8, angle=25.84825, rpm=2500.0),
    # uc.ShooterInfo(distance=6.80000001, angle=25.84825, rpm=2200.0),
    # uc.ShooterInfo(distance=7.0, angle=25.84825, rpm=2200.0),
    # uc.ShooterInfo(distance=7.5, angle=25.84825, rpm=2250.0),
    # uc.ShooterInfo(distance=8.0, angle=25.84825, rpm=2400.0),
    # uc.ShooterInfo(distance=8.5, angle=25.84825, rpm=2400.0),
    uc.ShooterInfo(distance=9.0, angle=25.84825, rpm=2500.0),
    uc.ShooterInfo(distance=9.5, angle=25.84825, rpm=2600.0),
    uc.ShooterInfo(distance=10.0, angle=25.84825, rpm=2700.0),
    uc.ShooterInfo(distance=10.5, angle=25.84825, rpm=2750.0),
    uc.ShooterInfo(distance=11.5, angle=25.84825, rpm=2900.0),
    uc.ShooterInfo(distance=12.5, angle=25.84825, rpm=3000.0),
    uc.ShooterInfo(distance=13.5, angle=25.84825, rpm=2900.0),
    uc.ShooterInfo(distance=15.5, angle=25.84825, rpm=3100.0),
    uc.ShooterInfo(distance=16.5, angle=25.84825, rpm=3200.0),
    uc.ShooterInfo(distance=17.5, angle=25.84825, rpm=3300.0),
]


def generate_speaker_distance_rpm():
    for info in _CHAMPS_TABLE_SPEAKER:
        rpm = info.rpm
        distance = info.distance
        print(f"speakerDistanceToRPM.put({distance}, {rpm});")


def generate_speaker_distance_angle():
    for info in _CHAMPS_TABLE_SPEAKER:
        gpos = uc.Point.from_tuple(nd._SPEAKER_POINT_1)
        rpos = uc.Point(9 - info.distance, 0)

        model = uc.Model(rpos, gpos, info.rpm)
        pm = uc.ProjectileMotion(nd._TIME_CHANGE, nd._USE_DRAG)

        distance = info.distance
        angle = round(
            uc.Vector.fromradians(
                uc.angle_search(model, pm, uc.PruneType.NONE, [gpos, uc.Point.from_tuple(nd._SPEAKER_POINT_2)], False)
            ),
            4,
        )

        print(f"speakerDistanceToAngle.put({distance}, {angle});")


def generate_floor_distance_rpm():
    for info in _CHAMPS_TABLE_FLOOR:
        rpm = info.rpm
        distance = info.distance
        print(f"floorSpotDistanceToRPM.put({distance}, {rpm});")


def generate_floor_distance_angle():
    for info in _CHAMPS_TABLE_FLOOR:
        gpos = uc.Point.from_tuple(nd._FLOOR_SPOT)
        rpos = uc.Point(9 - info.distance, 0)

        if info.distance > 5.5:
            do_height = True
        else:
            do_height = False

        model = uc.Model(rpos, gpos, info.rpm)
        pm = uc.ProjectileMotion(nd._TIME_CHANGE, nd._USE_DRAG)

        distance = info.distance
        angle = round(uc.Vector.fromradians(uc.angle_search(model, pm, uc.PruneType.NONE, [gpos], do_height)), 4)

        print(f"floorSpotDistanceToAngle.put({distance}, {angle});")


# generate_speaker_distance_angle()

generate_floor_distance_rpm()
generate_floor_distance_angle()
