#  from https://github.com/upul/CarND-Vehicle-Detection

import matplotlib.gridspec as gridspec
import matplotlib.image as img
import matplotlib.pyplot as plt
import numpy as np


def visualize_hog_features(hog_features, images, color_map=None, suptitle=None):
    num_images = len(images)
    space = gridspec.GridSpec(num_images, 2)
    space.update(wspace=0.1, hspace=0.1)
    plt.figure(figsize=(4, 2 * (num_images // 2 + 1)))

    for index in range(0, num_images * 2):
        if index % 2 == 0:
            axis_1 = plt.subplot(space[index])
            axis_1.axis('off')
            axis_1.imshow(images[index // 2], cmap=color_map)
        else:
            axis_2 = plt.subplot(space[index])
            axis_2.axis('off')
            axis_2.imshow(hog_features[index // 2], cmap=color_map)

    if suptitle is not None:
        plt.suptitle(suptitle)
    plt.show()


def display_random_images(image_files, num_of_images=12, images_per_row=6, main_title=None):
    random_files = np.random.choice(image_files, num_of_images)
    images = []
    for random_file in random_files:
        images.append(img.imread(random_file))

    grid_space = gridspec.GridSpec(num_of_images // images_per_row + 1, images_per_row)
    grid_space.update(wspace=0.1, hspace=0.1)
    plt.figure(figsize=(images_per_row, num_of_images // images_per_row + 1))

    for index in range(0, num_of_images):
        axis_1 = plt.subplot(grid_space[index])
        axis_1.axis('off')
        axis_1.imshow(images[index])

    if main_title is not None:
        plt.suptitle(main_title)
    plt.show()
