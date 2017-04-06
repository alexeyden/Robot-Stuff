#!/bin/env python3

import argparse
import sys
import sklearn.externals.joblib as joblib

import cProfile, pstats

from core.detector.classifier import *
from core.detector.detector import *

import matplotlib.pyplot as pyplot
import matplotlib.patches as pyplot_patches


def profile_start():
    pr = cProfile.Profile()
    pr.enable()
    return pr


def profile_end(pr):
    pr.disable()
    sortby = 'cumulative'
    ps = pstats.Stats(pr).sort_stats(sortby)
    ps.print_stats(20)


def get_classes(train_dir):
    classes = [d for d in os.listdir(train_dir) if os.path.isdir(os.path.join(train_dir, d))]
    return classes


def process_train(input_: str, output: str, profile=False):
    if profile:
        prof = profile_start()

    options = TrainingOptions(
        kmeans={'k_or_guess': 100},
        surf={'extended': True},
        svm={'cache_size': 1024}
    )

    results = MultipleClassTrainer(options, input_, dont_touch=['wheel']).train()
    joblib.dump(results, output)

    print('Training info has been written to {0}'.format(args.out_path))


def process_detect(train_data, input_, interactive=False, profile=False):
    training: TrainingResult = joblib.load(train_data)

    if input_ is None:
        import tkinter as tk
        master = tk.Tk()
        listbox = tk.Listbox(master)
        listbox.pack()
        for c in glob.glob("data/test/*.png"):
            listbox.insert(tk.END, os.path.basename(c))
        def on_click():
            nonlocal input_
            input_ = os.path.join('data/test/', listbox.selection_get())
            master.destroy()
        btn = tk.Button(master, text="OK", command=on_click)
        btn.pack()
        tk.mainloop()

    if input_ is None:
        sys.exit(-1)

    image = cv2.imread(input_)

    prof = profile_start() if profile else None

    fig, (ax1, ax2) = pyplot.subplots(2)
    ax1.imshow(image)

    for class_ in training.results.keys():
        if class_ == 'light':
            continue
        print(f'Running detect for {class_}')
        classifier = BagOfWordsClassifier(training.results[class_], training.options)
        detector = ScanningWindowDetector(classifier)
        matches, matrix = detector.detect(input_, interactive)

        ax2.imshow(matrix, interpolation='None', cmap='viridis')

        for x0y0, x1y1, prob in matches:
            ax1.add_patch(pyplot_patches.Rectangle(
                x0y0, x1y1[0] - x0y0[0], x1y1[1] - x0y0[1],
                fill=False,
                edgecolor="red"
            ))
            ax1.text(*x0y0, '{0} = {1:.3f}'.format(class_, prob), dict(backgroundcolor='red'))
            k=100
            print('{}: p={:.2f} c={},{} s={},{}'.format(
                class_, prob,
                (x1y1[0] + x0y0[0])/2.0/k, (x1y1[1] + x0y0[1])/2.0/k,
                (x1y1[0] - x0y0[0])/k, (x1y1[1] - x0y0[1])/k))

    if profile:
        profile_end(prof)

    pyplot.show()


def parse_args():
    parser = argparse.ArgumentParser(description='SupaVision Expert System')
    parser.add_argument('--profile', action='store_const', const=True, default=False)

    subparsers = parser.add_subparsers(help='Действие', dest='action')

    parser_train = subparsers.add_parser('train', help='Обучение')
    parser_train.add_argument('data_path', metavar="IN_IMAGES_DIR", help="путь до каталога с данными для обучения")
    parser_train.add_argument('out_path', metavar="OUT_TRAIN_FILE", help="выходной файл классификатора")

    parser_detect = subparsers.add_parser('detect', help='Поиск объектов')
    parser_detect.add_argument('train_data', metavar="TRAIN_DATA", help="файл классификатора")
    parser_detect.add_argument('input_image', metavar="IMAGE_PATH", help="входное изображение", nargs='?')
    parser_detect.add_argument('-i', dest='interact', help='Интерактивный режим', action='store_true')

    args = parser.parse_args()

    if args.action is None:
        parser.print_help()

    return args

args = parse_args()

if args.action == 'train':
    process_train(args.data_path, args.out_path, args.profile)
elif args.action == 'detect':
    process_detect(args.train_data, args.input_image, args.interact, args.profile)
