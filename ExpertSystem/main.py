#!/bin/env python3

import argparse
import sklearn.externals.joblib as joblib

import cProfile, pstats

from core.detector.trainer import *
from core.detector.classifier import *


def profile_start():
    pr = cProfile.Profile()
    pr.enable()
    return pr


def profile_end(pr):
    pr.disable()
    sortby = 'cumulative'
    ps = pstats.Stats(pr).sort_stats(sortby)
    ps.print_stats(20)


def process_train(input_, output, profile=False):
    if profile:
        prof = profile_start()

    options = TrainingOptions(kmeans={'k_or_guess': 20}, surf={})
    training = BagOfWordsTrainer(options, input_).train()

    if profile:
        profile_end(prof)

    joblib.dump(training, output)


def process_test(train_data, input_, profile=False):
    training = joblib.load(train_data)

    if profile:
        prof = profile_start()

    class_, dists = BagOfWordsClassifier(training).predict(input_)

    if profile:
        profile_end(prof)

    print('Class = {}'.format(class_))

    for class_, dist in dists.items():
        print('{} : {}'.format(class_, dist))


def parse_args():
    parser = argparse.ArgumentParser(description='SupaVision Expert System')
    parser.add_argument('--profile', action='store_const', const=True, default=False)

    subparsers = parser.add_subparsers(help='Действие', dest='action')

    parser_train = subparsers.add_parser('train', help='Обучение системы')
    parser_train.add_argument('data_path', metavar="INPUT_PATH", help="путь до каталога с данными для обучения")
    parser_train.add_argument('out_path', metavar="OUTPUT_PATH", help="выходной файл классификатора")

    parser_test = subparsers.add_parser('test', help='Классификация')
    parser_test.add_argument('train_data', metavar="TRAIN_DATA", help="файл классификатора")
    parser_test.add_argument('input_image', metavar="IMAGE_PATH", help="входное изображение")

    args = parser.parse_args()

    if args.action is None:
        parser.print_help()

    return args

args = parse_args()

if args.action == 'train':
    process_train(args.data_path, args.out_path, args.profile)
    print('Training info has been written to {0}'.format(args.out_path))
elif args.action == 'test':
    process_test(args.train_data, args.input_image, args.profile)
