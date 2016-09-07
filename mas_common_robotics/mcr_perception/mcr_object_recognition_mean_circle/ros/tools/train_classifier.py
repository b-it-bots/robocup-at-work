#!/usr/bin/env python


import roslib
import sys
import os.path
import argparse
from mcr_object_recognition_mean_circle.svm_trainer import SVMTrainer

PACKAGE = 'mcr_object_recognition_mean_circle'
DATA_PACKAGE = 'mds_pointclouds'
sys.path.append(os.path.join(roslib.packages.get_pkg_dir(PACKAGE), 'common', 'src'))


#  ./train_classifier.py --dataset objects_daylight --objects F20_20_G S40_40_G --output <name of classifier>
#  ./train_classifier.py --dataset objects_daylight --output <name of classifier>

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='''
    Train and save SVM classifier for object recognition.
    The classifier will be saved to the "common/config/" folder.
    ''')
    parser.add_argument('--dataset', help='dataset to use',
                        default='objects_laying')
    parser.add_argument('--objects', help='list of objects to use',
                        nargs='*', default='all')
    parser.add_argument('--output', help='output filename (without folder and '
                        'extension, default: "classifier")', default='classifier')
    args = parser.parse_args()
    data_folder = os.path.join(roslib.packages.get_pkg_dir(DATA_PACKAGE), 'objects', 'workspace_setups', args.dataset)
    cfg_folder = os.path.join(roslib.packages.get_pkg_dir(PACKAGE), 'common', 'config')

    trainer = SVMTrainer(data_folder)
    classifier = trainer.train(args.objects)
    directory = os.path.join(cfg_folder, args.output)
    print "Saving classifer to ", directory
    if not os.path.exists(directory):
        print "Creating directory ", directory
        os.makedirs(directory)
    classifier.save(os.path.join(cfg_folder, args.output, 'classifier.pkl'),
                    os.path.join(cfg_folder, args.output, 'label_encoder.pkl'))
