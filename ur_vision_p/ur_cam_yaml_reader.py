#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy
from numpy import matlib,linalg

#import rospy
import yaml,os,sys

class camYamlReader():

    def __init__(self ):
        pass

    """
    get instrinc param from yaml file
    """
    def get_instrinc_data(self):
        f = open("../yaml/fixed_cam_param.yaml")
        yamldata = yaml.load(f)
        #print yamldata
        instrinc_param =  yamldata['ins_matrix']
        f.close()
        return instrinc_param

    """
        get instrinc param detail('kx': kx, 'ky': ky, "u0": u0, "v0": v0) from yaml file
    """
    def get_instrinc_param_detail(self):
        f = open("../yaml/fixed_cam_param.yaml")
        yamldata = yaml.load(f)
        # print yamldata
        kx = yamldata['kx']
        # print kx
        ky = yamldata['ky']
        # print ky
        u0 = yamldata['u0']
        v0 = yamldata['v0']
        focallength=yamldata["focal_length"]
        instrinc_param_detail = {'kx': kx, 'ky': ky, "u0": u0, "v0": v0,"focal_length":focallength}
        f.close()
        return instrinc_param_detail
    def get_extrinc_param(self):
        f = open("../yaml/fixed_cam_param.yaml")
        yamldata = yaml.load(f)
        X = yamldata["X"]
        f.close()
        return X

def get_in_ex_cam_param_detail():
        path = os.path.realpath(__file__)[0]
        sys.path.insert(0, path)
        yamlreader = camYamlReader()
        in_param = yamlreader.get_instrinc_data()
        ex_param = yamlreader.get_extrinc_param()
        param_detail=yamlreader.get_instrinc_param_detail()

        return in_param,param_detail,ex_param


def main():
   print  get_in_ex_cam_param_detail()

if __name__ == "__main__":
    main()