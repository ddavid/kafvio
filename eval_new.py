import operator
import os
import re
import subprocess
import sys
from tqdm import tqdm
import pandas as pd
from collections import OrderedDict
from pprint import pprint

tqdm.monitor_interval = 0

WEIGHT_BACKUP_DIR = '/home/fsd/Documents/github-repos/fsd-darknet/backup'
MODEL_DIR = '/home/fsd/Documents/github-repos/fsd-darknet/cfg/eval'
DATA_FILE = '/home/fsd/Documents/github-repos/fsd-darknet/data/ultra-convergence_only-red-cones.data'
EVAL_FILE = '/home/fsd/Documents/github-repos/fsd-darknet/evaluations/2018-05-10_no-big-red_tiny-yolo-voc_25-grid.csv'

DARKNET_EXEC = '/home/fsd/Documents/github-repos/fsd-darknet/darknet'

BACKUP_INTERVAL = 10
WEIGHT_INTERVAL = 500

# This allows you to skip through the first evaluation steps if the script crashed, however
# this will only work if you didn't add any new configs / weights!
SKIP = 0

# Compile all expressions as they will be used a lot :)
expressions = [
    r"(detections_count) = (\d*)",
    r"(unique_truth_count) = (\d*)",
    r"(yellow-cone), \t ap = (\d{1,3}.\d{2})",
    r"(blue-cone), \t ap = (\d{1,3}.\d{2})",
    r"((?<!big-)red-cone), \t ap = (\d{1,3}.\d{2})",
    r"(big-red-cone), \t ap = (\d{1,3}.\d{2})",
    r"(precision) = (\d{1}.\d{2})",
    r"(recall) = (\d{1}.\d{2})",
    r"(F1-score) = (\d{1}.\d{2})",
    r"(TP) = (\d*)",
    r"(FP) = (\d*)",
    r"(FN) = (\d*)",
    r"(average IoU) = (\d{2}.\d{2})",
    r"(\(mAP\)) = (\d{1}.\d*)"
]

matchers = [re.compile(expression) for expression in expressions]


def get_files(directory, postfix):
    """
        Returns all files in the 'directory' with the specifies postfix
        It's sorted to ensure determinism
    """
    filenames = os.listdir(directory)
    return sorted([filename for filename in filenames if filename.endswith(postfix)])


config_files = get_files(MODEL_DIR, 'cfg')
weights = get_files(WEIGHT_BACKUP_DIR, 'weights')

backup_counter = 0

result_list = []

num_weights = len(weights)

for config_file in tqdm(config_files, desc='Model Loop'):
    pattern = re.escape(config_file.replace('.cfg', '')) + r"_(\d+).weights"
    matcher = re.compile(pattern)

    # print matcher.findall("yolov2-tiny-5_custom_anchors_low_lr_1000.weights")

    validations = {}

    for weight in weights:
        matched_weight = matcher.findall(weight)

        if matched_weight:
            validations[int(matched_weight[0])] = weight

    # corresponding_weights = [weight for weight in weights if weight.startswith(config_file.replace('.cfg', ''))]

    # print corresponding_weights
    validations = OrderedDict(sorted(validations.items()))

    if not validations:
        print("Config file has no weights: %s" % config_file)
    else:
        for k, v in tqdm(validations.items(), desc='Weight Loop for %s' % config_file):
            if k % WEIGHT_INTERVAL == 0 or 'final' in v:
                if SKIP > 0:
                    SKIP -= 1
                    print("Skipping ... ")
                    continue

                # print("Running validation for %s -> %s" % (config_file, v))

                output = subprocess.check_output([DARKNET_EXEC, 'detector', 'map',
                                                  DATA_FILE,
                                                  os.path.join(MODEL_DIR, config_file),
                                                  os.path.join(WEIGHT_BACKUP_DIR, v)], stderr=subprocess.STDOUT)

                results = {
                    'model': config_file,
                    'weights': v
                }

                print output
                for matcher in matchers:
                    matches = matcher.findall(output)
                    if matches:
                      print matches
                      results[matches[0][0]] = matches[0][1]

                if results:
                  result_list.append(results)

                backup_counter += 1

                if backup_counter % BACKUP_INTERVAL == 0:
                    df = pd.DataFrame(result_list)

                    df.to_csv(EVAL_FILE, columns=[
                        'model',
                        'weights',
                        '(mAP)',
                        'F1-score',
                        'precision',
                        'recall',
                        'TP',
                        'FP',
                        'FN',
                        'average IoU',
                        'detections_count',
                        'unique_truth_count',
                        'yellow-cone',
                        'blue-cone',
                        'red-cone',
                        'big-red-cone'
                    ])
                    # print("Writing to file ... ")

