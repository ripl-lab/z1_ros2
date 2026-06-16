#!/usr/bin/env python3
import sys
import time

# Ensure your python path can find the Unitree SDK if needed
# import z1_sdk 

def main():
    print("Starting waypoint script...")
    
    # Paste your recorded coordinates here!
    # q: [joint1, joint2, joint3, joint4, joint5, joint6]
    pick_location  = [0.00190,  0.00365, -0.02694, -0.09964, -0.00084, -0.00276]
    place_location = [0.524, -0.312, 0.891, 0.102, -0.054, 0.231] 
    
    # Your arm initialization and movement code goes here
    # arm = z1_sdk.UnitreeArm()
    # arm.moveJ(pick_location, 0.5)
    
    print("Movement complete!")

if __name__ == '__main__':
    main()
