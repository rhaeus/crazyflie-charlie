#!/usr/bin/env python


def signIdConverter(id):

    if id == 0: 
        signName = "no_bicycle"

    if id == 1: 
        signName = "airport"

    if id == 2: 
        signName = "dangerous_left"

    if id == 3: 
        signName = "dangerous_right"

    if id == 4: 
        signName = "follow_left"

    if id == 5: 
        signName = "follow_right"

    if id == 6: 
        signName = "junction"

    if id == 7: 
        signName = "no_heavy_truck"

    if id == 8: 
        signName = "no_parking"

    if id == 9: 
        signName = "no_stopping_and_parking"

    if id == 10: 
        signName = "residential"

    if id == 11: 
        signName = "narrows_from_left"

    if id == 12: 
        signName = "narrows from right"

    if id == 13: 
        signName = "roundabout"

    if id == 14: 
        signName = "stop"

    return signName



if __name__ == "__main__":
    main()

