#!/usr/bin/env python

import yaml
import numpy as np

from pybrain.tools.customxml.networkwriter import NetworkWriter
from pybrain.tools.customxml.networkreader import NetworkReader


class ClassificationNetwork:
    def __init__(self, net, labels, mean, std):
        self.net = net
        self.labels = labels
        self.mean = mean
        self.std = std

    def save(self, filename, desc=None):
        NetworkWriter.writeToFile(self.net, filename + '.xml')
        params = {'labels': self.labels,
                  'mean': self.mean.tolist(),
                  'std': self.std.tolist()}
        with open(filename + '.yaml', 'w') as f:
            f.write(yaml.dump(params, default_flow_style=False))

    def classify(self, sample):
        klass = np.argmax(self.net.activate(self.standardize(sample)))
        label = self.labels[klass]
        return (klass, label)

    def standardize(self, sample):
        return (sample - self.mean) / self.std

    @classmethod
    def load(cls, filename):
        net = NetworkReader.readFrom(filename + '.xml')
        params = yaml.load(open(filename + '.yaml', 'r'))
        mean = np.array(params['mean'])
        std = np.array(params['std'])
        return ClassificationNetwork(net, params['labels'], mean, std)
