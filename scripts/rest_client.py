#!/usr/bin/env python3
import os
from flask import Flask, jsonify
from flask_restful import Resource, Api, reqparse
import pandas as pd
from geometry_msgs.msg import Point, Quaternion
import rospy
import interactive_commands as ic

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


app = Flask(__name__)
api = Api(app)


@app.route("/")
def get_hello_world():
    return "Hello, World!"


@app.route("/move_forward")
def move_pr2_forward():
    response = ic.move_forward()
    if response is not None:
        return jsonify("successfully moved forward"), 200
    else:
        return jsonify(response), 400


@app.route("/move_backward")
def move_pr2_backward():
    response = ic.move_backward()
    if response is not None:
        return jsonify("successfully moved backward"), 200
    else:
        return jsonify(response), 400


@app.route("/follow_human_right_hand")
def follow_human_right_hand():
    pose1 = Point(2.0,2.0,1.0)
    pose2 = Point(2.1,2.1,1.1)
    # TODO: next step here is to find all the trajectory points that human hand has followed while doping pouring. 
    #  or atlest at start of each events. Then pass that list of poses. In the metod inside interactive_commands.py you 
    #  can calculate the difference for each subsequent point. And then giskard_wrapper.add_cmd() to giskard so 
    #  that the difference between those points is saved into giskard and let giskard calulate trajectory for pr2 hands.
    ori2 = Quaternion(0, 0, 0, 1)
    response = ic.follow_human_right_hand()
    if response is not None:
        return jsonify("successfully followed the trajectory"), 200
    else:
        return jsonify(response), 400


@app.route("/reset")
def reset():
    response = ic.reset()
    if response is not None:
        return jsonify("successfully reset the environment"), 200
    else:
        return jsonify(response), 400


@app.route("/readHandPosesFromJsonFile")
def readHandPosesFromJsonFile():
    response = ic.readHandPosesFromJsonFile()
    if response is not None:
        return jsonify("successfully reset the environment"), 200
    else:
        return jsonify(response), 400


@app.route("/perform_pouring")
def perform_pouring():
    response = ic.perform_pouring()
    if response is not None:
        return jsonify("successfully pouring performed"), 200
    else:
        return jsonify(response), 400
    

