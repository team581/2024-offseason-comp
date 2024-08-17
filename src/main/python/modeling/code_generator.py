import used_classes as uc
import numerical_data as nd
import math
import dataclasses

_CHAMPS_TABLE_SPEAKER = [uc.ShooterInfo(distance= 1.38, angle= 58.1, rpm= 3000.0), #0
                         uc.ShooterInfo(distance= 2.16, angle= 47.8, rpm= 3000.0),
                         uc.ShooterInfo(distance= 2.5, angle= 42.0, rpm= 4000.0),
                         uc.ShooterInfo(distance= 3.5, angle= 33.9635, rpm= 4000.0),
                         uc.ShooterInfo(distance= 4.5, angle= 28.20125, rpm= 4000.0),
                         uc.ShooterInfo(distance= 5.5, angle= 25.84825, rpm= 4000.0),
                         uc.ShooterInfo(distance= 6.5, angle= 21.30525, rpm= 4800.0),
                         uc.ShooterInfo(distance= 7.5, angle= 20.27075, rpm= 4800.0),
                         uc.ShooterInfo(distance= 9.0, angle= 18.7305, rpm= 4800.0) #8
                         ]
_CHAMPS_TABLE_FLOOR = [ uc.ShooterInfo(distance= 0.0 , angle= 58.1, rpm= 1000.0), #0
                        uc.ShooterInfo(distance= 1.0, angle= 47.8, rpm= 1000.0),
                        uc.ShooterInfo(distance= 1.2, angle= 42.0, rpm= 1500.0),
                        uc.ShooterInfo(distance= 3.0, angle= 33.9635, rpm= 1800.0),
                        uc.ShooterInfo(distance= 5.8, angle= 28.20125, rpm= 2700.0),
                        uc.ShooterInfo(distance= 6.5, angle= 25.84825, rpm= 2700.0),
                        uc.ShooterInfo(distance= 500.0, angle= 21.30525, rpm= 2700.0),
                        uc.ShooterInfo(distance= 581.0, angle= 20.27075, rpm= 3200.0)
                        ]

def generate_speaker_distance_rpm():
    for info in _CHAMPS_TABLE_SPEAKER:
        rpm = info.rpm
        distance = info.distance
        print(f'speakerDistanceToRPM.put({distance}, {rpm});')

def generate_speaker_distance_angle():
    for info in _CHAMPS_TABLE_SPEAKER:
        gpos = uc.Point(9,nd._SPEAKER_HEIGHT)
        rpos = uc.Point(9-info.distance,0)

        model = uc.Model(rpos, gpos, info.rpm)
        pm = uc.ProjectileMotion(nd._TIME_CHANGE, nd._USE_DRAG)

        distance = info.distance
        angle = round(uc.Vector.fromradians(uc.get_angle_better(model,pm)),4)

        print(f'speakerDistanceToAngle.put({distance}, {angle});')

def generate_floor_distance_rpm():
    for info in _CHAMPS_TABLE_FLOOR:
        rpm = info.rpm
        distance = info.distance
        print(f'floorSpotDistanceToRPM.put({distance}, {rpm});')

def generate_floor_distance_angle():
    for info in _CHAMPS_TABLE_FLOOR:
        gpos = uc.Point(9,nd._FLOOR_SPOT_HEIGHT)
        rpos = uc.Point(9-info.distance,0)

        model = uc.Model(rpos, gpos, info.rpm)
        pm = uc.ProjectileMotion(nd._TIME_CHANGE, nd._USE_DRAG)

        distance = info.distance
        angle = round(uc.Vector.fromradians(uc.get_angle_floor(model,pm)),4)

        print(f'floorSpotDistanceToAngle.put({distance}, {angle});')



# generate_speaker_distance_angle()
generate_floor_distance_rpm()
generate_floor_distance_angle()
