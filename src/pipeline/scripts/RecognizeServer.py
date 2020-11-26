#!/usr/bin/python
# coding: utf-8
import rospy

# from hik_ptz_camera.srv import PtzCtrl
import SocketServer
import struct
import json
import threading
import socket
import copy


class RecognizeServer:
    def __init__(self):
        HOST, PORT = "0.0.0.0", 20008
        self.sock_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock_server.bind((HOST, PORT))
        self.sock_server.listen(1)
        self.latest_recognize_res = None
        self.server_thread = threading.Thread(target=self.server_up)
        self.server_thread.setDaemon(True)
        self.server_thread.start()

    def server_up(self):
        print "waiting for conneced..."
        self.connection, address = self.sock_server.accept()
        print address, " connected."
        while True:
            self.data = self.connection.recv(1024)
            if not self.data:
                print "empty data"
                break
            try:
                json_dict = json.loads(self.data)
                self.latest_recognize_res = json_dict
            except:
                print "Invalid msg type.Not json format."
                print self.data

        self.connection.close()
        self.sock_server.close()

    def get_latest_result(self):
        rospy.sleep(0.05)
        if self.latest_recognize_res:
            res = copy.deepcopy(self.latest_recognize_res)
            self.latest_recognize_res = None
            return res

    def object_fastgo(self, object_index):
        rospy.sleep(0.2)
        print "Object position: ", object_index
        goal = self.get_latest_result()
        print goal
        if goal and goal[0]["xmax"]:
            x = (goal[0]["xmin"] + goal[0]["xmax"]) / 2
            y = (goal[0]["ymin"] + goal[0]["ymax"]) / 2
            print "fast_go"
            self.fast_go(x, y)
        import pdb

        pdb.set_trace()

    def test(self):
        # self.server_thread.start()
        while not rospy.is_shutdown():
            rospy.sleep(2.5)
            print "trigger new target position."
            goal = self.get_latest_result()
            print goal
            if goal and goal[0]["xmax"]:
                x = (goal[0]["xmin"] + goal[0]["xmax"]) / 2
                y = (goal[0]["ymin"] + goal[0]["ymax"]) / 2
                print "fast_go"
                # self.fast_go(x,y)

    def fast_go(self, x, y):
        print "x:%d,y:%d" % (x, y)
        param_1, param_2 = x, y
        print "param_1:%d,param_2:%d" % (param_1, param_2)
        rospy.wait_for_service("ptz_ctrl")
        try:
            ptz_ctrl = rospy.ServiceProxy("ptz_ctrl", PtzCtrl)
            resp1 = ptz_ctrl(1, int(param_1), int(param_2), 0)
            print resp1.status_message
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e


recognize_server = RecognizeServer()

if __name__ == "__main__":
    rospy.init_node("nuc_tx2_test")
    rs = RecognizeServer()
    rs.test()
    # rs.run()
