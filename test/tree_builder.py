from collections import defaultdict
from math import inf, log2
import os
from random import Random, randint
import sys
from time import time
import cv2
import pickle
import fast12
from array import array
import datetime
from pympler import tracker
import gc
from collections import OrderedDict

radius = 3
pixels = [(-3, 0),(-3, 1),(-2, 2),(-1, 3),(0, 3),(1, 3),(2, 2),(3, 1),(3, 0),(3, -1),(2, -2),(1, -3),(0, -3),(-1, -3),(-2, -2),(-3, -1)]
threshold = 25
continous_pixels = 9

depths = []
min_idxs = []

class Node:
    def __init__(self, data, available_feature, feature, depth) -> None:
        self.data_ = data
        self.available_feature_ = available_feature
        self.feature_ = feature
        self.children_ = {}
        self.result_ = -1
        self.depth_ = depth


# 0:not corner, 1: darker corner, 2: brighter corner
def fast(image, row, col):
    darker = 0
    lighter = 0

    # First detect pixel on the circle at horizontal and vertical direction
    # And then find whether there has three pixels darker or lighter than other one pixel.
    pixel_1 = image[row + pixels[0][0]][col + pixels[0][1]]
    pixel_4 = image[row + pixels[4][0]][col + pixels[4][1]]
    pixel_8 = image[row + pixels[8][0]][col + pixels[8][1]]
    pixel_12 = image[row + pixels[12][0]][col + pixels[12][1]]

    center = image[row][col]
    center_upper_threshold = center + threshold
    center_lower_threshold = center - threshold

    # Collect all pixels on the circle into vectoe and
    # check whether there are 12 continuous pixels darker or lighter.
    around_pixels = []
    around_pixels_cmp = []
    for pixel in pixels:
        around_pixels.append((image[row + pixel[0], col + pixel[1]]))
        if image[row + pixel[0], col + pixel[1]] > center_upper_threshold:
            around_pixels_cmp.append(2)
        elif image[row + pixel[0], col + pixel[1]] < center_lower_threshold:
            around_pixels_cmp.append(1)
        else:
            around_pixels_cmp.append(0)
    # around_pixels.reverse()
    # around_pixels_cmp.reverse()

    # if pixel_1 >= center_upper_threshold:
    #     lighter += 1
    # elif pixel_1 <= center_lower_threshold:
    #     darker += 1
    # if pixel_8 >= center_upper_threshold:
    #     lighter += 1
    # elif pixel_8 <= center_lower_threshold:
    #     darker += 1

    # if not (darker == 2 or lighter == 2):
    #     return (0, array('b', around_pixels_cmp))

    # if (pixel_4 >= center_upper_threshold):
    #     lighter += 1
    # elif (pixel_4 <= center_lower_threshold):
    #     darker += 1
    # if (pixel_12 >= center_upper_threshold):
    #     lighter += 1
    # elif (pixel_12 <= center_lower_threshold):
    #     darker += 1

    # if not (darker >= 3 or lighter >= 3):
    #     return (0, array('b', around_pixels_cmp))

        # # Use double pointer to find the continuous subset.
        # start = 0; end = 0; forward = 0; count = 0

        # while forward < len(pixels):
        #     # The end moves forward if start pixel and end pixel are samely darker of lighter
        #     if (around_pixels[start] <= center_lower_threshold and around_pixels[end] <= center_lower_threshold) or (around_pixels[start] >= center_upper_threshold and around_pixels[end] >= center_upper_threshold):
        #         end+= 1
        #         if end >= len(around_pixels):
        #             end = end % len(around_pixels)

        #         count+= 1
        #         if count >= continous_pixels:
        #             # darker
        #             if around_pixels[start] <= center_lower_threshold:
        #                 return (1, array('b', around_pixels_cmp))
        #             # brighter
        #             if around_pixels[start] >= center_upper_threshold:
        #                 return (2, array('b', around_pixels_cmp))
        #     else:
        #         if start == end:
        #             end+= 1
        #         if end >= start:
        #             forward += end - start
        #         else:
        #             forward += end - start + len(around_pixels)
        #         if end >= len(around_pixels):
        #             end = end % len(around_pixels)

        #         start = end
        #         count = 0

        # return (0, array('b', around_pixels_cmp))
    
    num_consecutive = 0
    first_cons = 0
    val = 0
        
    for i in range(16):
        val = image[row + pixels[i][0], col + pixels[i][1]]
        if val > center_upper_threshold:
            num_consecutive += 1
            if num_consecutive == continous_pixels:
                return (2, array('b', around_pixels_cmp), row, col)  
        else:
            if num_consecutive == i:
                first_cons = i
            num_consecutive = 0
    if first_cons + num_consecutive >= continous_pixels:
        return (2, array('b', around_pixels_cmp), row, col) 

    num_consecutive = 0
    first_cons = 0
    val = 0
        
    for i in range(16):
        val = image[row + pixels[i][0], col + pixels[i][1]]
        if val < center_lower_threshold:
            num_consecutive += 1
            if num_consecutive == continous_pixels:
                return (1, array('b', around_pixels_cmp), row, col)  
        else:
            if num_consecutive == i:
                first_cons = i
            num_consecutive = 0
    if first_cons + num_consecutive >= continous_pixels:
        return (1, array('b', around_pixels_cmp), row, col)  
    
    return (0, array('b', around_pixels_cmp), row, col) 


def detectCorners(images):
    # Check every pixel and find if there are 12 continuous darker or lighter
    # pixels and store its pixel values each.
    all_corners = []
    print('Has ', len(images), ' images......')
    count = 0
    for (img_idx, image) in enumerate(images):
        print('Detect ', img_idx, ' image......')
        corner_img = []
        for row in range(image.shape[0]):
            for col in range(image.shape[1]):
                if row <= radius or abs(image.shape[0] - row) <= radius \
                    or col <= radius or abs(image.shape[1] - col) <= radius:
                    continue
                if count == 9:
                    a = 1
                count += 1
                corner_img.append(fast(image, row, col))
        all_corners.append({'image': img_idx, 'corners': corner_img})

    return all_corners
    
# # Compute the entropy of specific bit, like pixel_1 is brighter than center
# # features should be in the same group and have same feature at specific bit.
# def computeEntropy(features):
#     fea_value = defaultdict(lambda: 0)
#     for fea in features:
#         fea_value[fea[0]] += 1
    
#     total = 0
#     for value in fea_value.values():
#         total += value
    
#     total_entropy = 0
#     for value in fea_value.values():
#         total_entropy -= (value / total) *log2(value / total)

#     return total_entropy

        # separated_value = {
        #     'pos': {
        #         'number':0,
        #         'is_a_corner':0,
        #         'not_a_corner':0
        #     },
        #     'neg': {
        #         'number':0,
        #         'is_a_corner':0,
        #         'not_a_corner':0
        #     },
        #     'neither': {
        #         'number':0,
        #         'is_a_corner':0,
        #         'not_a_corner':0
        #     }
        # }
def computeEntropy(feature_numbers):
    # if feature_numbers['is_a_corner'] == 0 and feature_numbers['not_a_corner'] == 0:
    #     return 0
    # elif feature_numbers['not_a_corner'] == 0:
    #     return feature_numbers['number'] * (-feature_numbers['is_a_corner'] / feature_numbers['number'] *log2(feature_numbers['is_a_corner'] / feature_numbers['number']) )
    # elif feature_numbers['is_a_corner'] == 0:
    #     return feature_numbers['number'] * (-feature_numbers['not_a_corner'] / feature_numbers['number'] *log2(feature_numbers['not_a_corner'] / feature_numbers['number']))
    # return feature_numbers['number'] * (-feature_numbers['is_a_corner'] / feature_numbers['number'] *log2(feature_numbers['is_a_corner'] / feature_numbers['number']) - feature_numbers['not_a_corner'] / feature_numbers['number'] *log2(feature_numbers['not_a_corner'] / feature_numbers['number']))
    if feature_numbers['is_a_corner'] == 0 and feature_numbers['not_a_corner'] == 0:
        return 0
    elif feature_numbers['not_a_corner'] == 0:
        return (-feature_numbers['is_a_corner'] / feature_numbers['number'] *log2(feature_numbers['is_a_corner'] / feature_numbers['number']) )
    elif feature_numbers['is_a_corner'] == 0:
        return (-feature_numbers['not_a_corner'] / feature_numbers['number'] *log2(feature_numbers['not_a_corner'] / feature_numbers['number']))
    return (-feature_numbers['is_a_corner'] / feature_numbers['number'] *log2(feature_numbers['is_a_corner'] / feature_numbers['number']) - feature_numbers['not_a_corner'] / feature_numbers['number'] *log2(feature_numbers['not_a_corner'] / feature_numbers['number']))
    



# corners value is [positive(negtive, neither), [value at each pixel]]
def build(node):

    depths.append(node.depth_)
    
    # if len(node.data_) == 0:
    #     node.result_ = -1
    #     return
    
    # print('data_ size is ', sys.getsizeof(node.data_))
    # test if the node has the same result
    darker = 0; neither = 0; brighter = 0
    for corner in node.data_:
        if corner[0] == 0:
            neither += 1
        elif corner[0] == 1:
            darker += 1
        else:
            brighter += 1
    if darker == len(node.data_):
        del node.data_
        node.result_ = 1
        return
    elif brighter == len(node.data_):
        del node.data_
        node.result_ = 2
        return
    elif neither == len(node.data_):
        del node.data_
        node.result_ = 0
        return
    elif node.depth_ == len(node.data_[0][1]):
        del node.data_
        node.result_ = 0
        return
    size = len(node.data_[0][1])
    # find max entropy in all features
    # iterate all features.
    # for one value of one feature, count its result and calculate its entropy.
    # add all entropy of different value
    all_entropy = []
    each_feature_separaed_value_count = {}
    for i in range(size):
        if node.available_feature_[i] == 0:
            all_entropy.append(-1)
            continue
        separated_value = {
            'pos': {
                'number':0,
                'is_a_corner':0,
                'not_a_corner':0
            },
            'neg': {
                'number':0,
                'is_a_corner':0,
                'not_a_corner':0
            },
            'neither': {
                'number':0,
                'is_a_corner':0,
                'not_a_corner':0
            }
        }
        for idx, corner in enumerate(node.data_):
            if corner[1][i] == 0:
                separated_value['neither']['number'] += 1
                if corner[0] == 0:
                    separated_value['neither']['not_a_corner'] += 1
                else:
                    separated_value['neither']['is_a_corner'] += 1

            elif corner[1][i] == 1:
                separated_value['neg']['number'] += 1
                if corner[0] == 0:
                    separated_value['neg']['not_a_corner'] += 1
                else:
                    separated_value['neg']['is_a_corner'] += 1
            else:
                separated_value['pos']['number'] += 1
                if corner[0] == 0:
                    separated_value['pos']['not_a_corner'] += 1
                else:
                    separated_value['pos']['is_a_corner'] += 1
        each_feature_separaed_value_count[i] = separated_value
        separated_value_entropy = []
        each_total = []
        total = 0
        for key in separated_value.keys():
            separated_value_entropy.append(computeEntropy(separated_value[key]))
            each_total.append(separated_value[key]['number'])
            total += separated_value[key]['number']
        weighted_entropy_sum = 0
        for i in range(len(each_total)):
            weighted_entropy_sum += (each_total[i] / total) *  separated_value_entropy[i]
        
        all_entropy.append(weighted_entropy_sum)


    # print('data_ size after cleaning... ', sys.getsizeof((node.data_))) 
    max_idx = all_entropy.index(max(all_entropy))
    min_idxs.append(max_idx)
    node.feature_ = max_idx
    available_features = node.available_feature_.copy()
    available_features[max_idx] = 0

    min_feature_separated_value_corner = defaultdict(lambda: [])
    for corner in node.data_:
        min_feature_separated_value_corner[corner[1][max_idx]].append(corner)
    min_feature_separated_value_corner = OrderedDict(sorted(min_feature_separated_value_corner.items(), reverse=False))
    del node.data_
    min_feature_separated_value_count = each_feature_separaed_value_count[max_idx]


    for key in min_feature_separated_value_corner.keys():
        # count_key = ''
        # if key == 0:
        #     count_key = 'neither'
        # elif key == 1:
        #     count_key = 'neg'
        # else:
        #     count_key = 'pos'
        # count = min_feature_separated_value_count[count_key]
        # if count['number'] == count['is_a_corner']:
        #     child = Node(data=min_feature_separated_value_corner[key], available_feature=available_features, feature=-1, depth=node.depth_ + 1)
        #     child.result_ = 1
        #     node.children_[key] = {'feature': node.feature_ , 'value': key, 'node': child}
        # elif 0 == count['is_a_corner']:
        #     child = Node(data=min_feature_separated_value_corner[key], available_feature=available_features, feature=-1, depth=node.depth_ + 1)
        #     child.result_ = 0
        #     node.children_[key] = {'feature': node.feature_ , 'value': key, 'node': child} 
        # else:
        #     child = Node(data=min_feature_separated_value_corner[key], available_feature=available_features, feature=-1, depth=node.depth_ + 1)
        #     node.children_[key] = {'feature': node.feature_ , 'value': key, 'node': child}
        #     build(child)
        child = Node(data=min_feature_separated_value_corner[key], available_feature=available_features, feature=-1, depth=node.depth_ + 1)
        node.children_[key] = {'feature': node.feature_ , 'value': key, 'node': child}
        build(child)
    


def readAllImages(dir_path, random=False, size=-1):
    
    onlyfiles = [os.path.join(dir_path, f) for f in os.listdir(dir_path) if os.path.isfile(os.path.join(dir_path, f)) and f.lower().endswith(('.png', '.jpg', '.jpeg', '.tiff', '.bmp', '.gif'))]
    images = None
    if size == -1:
        images = [cv2.imread(f, cv2.IMREAD_GRAYSCALE) for f in onlyfiles]
    else:
        if random:
            if size < len(onlyfiles):
                images = []
                unavailable = []
                for i in range(size):
                    idx = randint(0, len(onlyfiles))
                    while idx in unavailable:
                        idx = randint(0, len(onlyfiles))
                    unavailable.append(idx)
                    images.append(cv2.imread(onlyfiles[idx], cv2.IMREAD_GRAYSCALE)) 
            else:
                images = [cv2.imread(f, cv2.IMREAD_GRAYSCALE) for f in onlyfiles]
        else:
            if size < len(onlyfiles):
                images = []
                for  i in range(size):
                    images.append(cv2.imread(onlyfiles[i], cv2.IMREAD_GRAYSCALE))
            else:
                images = [cv2.imread(f, cv2.IMREAD_GRAYSCALE) for f in onlyfiles]
    return images


if __name__ == '__main__':
    images_size = 50
    all_images = readAllImages('/home/yeren/rgbd_dataset_freiburg1_desk/data', random=True, size=images_size)
    # all_images = readAllImages('/home/yeren/Simple-SLAM/test', random=True, size=images_size)
    corners = detectCorners(all_images)
    # corners = detectCorners(readAllImages('/home/yeren/Simple-SLAM/test')[0:1])
    # print("All corners cost memory ", sys.getsizeof(corners), "bytes.")


    # save corners
    with open('all_corners_'+ str(images_size) + '.pkl', 'wb') as outp:
        pickle.dump(corners, outp, pickle.HIGHEST_PROTOCOL)
    corners = None
    with open('all_corners_'+ str(images_size) + '.pkl', 'rb') as inp:
        corners = pickle.load(inp)

    all_corners = []
    for img_corner in corners:
        all_corners.extend(img_corner['corners'])
    root = Node(data=all_corners, available_feature=[1 for i in range(len(all_corners[0][1]))], feature=-1, depth=0)

    print('Build Tree......')
    build(root)
    print('Finish Building Tree......')

    # count = 0
    # for i in range(len(min_idxs)):
    #     print(min_idxs[i], end=" ")
    #     count += 1
    #     if count % 50 == 0:
    #         print()
    # # print("depth size is ", len(depths))
    # count = 0
    # for i in range(len(min_idxs)):
    #     print(min_idxs[i], end=" ")
    #     count += 1
    #     if count % 50 == 0:
    #         print()
    # print("min_idxs size is ", len(min_idxs))

    # # save root
    # with open('root'+ str(images_size) + '.pkl', 'wb') as outp:
    #     pickle.dump(root, outp, pickle.HIGHEST_PROTOCOL)
    # root = None
    # with open('root'+ str(images_size) + '.pkl', 'rb') as inp:
    #     root = pickle.load(inp)

    image = readAllImages('/home/yeren/Simple-SLAM/test')[0]
    # image = readAllImages('/home/yeren/Downloads')[0]
    # image = cv2.imread("/home/yeren/dataset/sequences/05/image_0/000000.png", cv2.IMREAD_GRAYSCALE)
    circle = 0
    corners = []
    count = 0
    time_for_node_traverse = datetime.datetime.now()
    for row in range(image.shape[0]):
        for col in range(image.shape[1]):
            if row <= radius or abs(image.shape[0] - row) <= radius \
                    or col <= radius or abs(image.shape[1] - col) <= radius:
                continue

            count +=1

            center = image[row][col]
            center_upper_threshold = center + threshold
            center_lower_threshold = center - threshold

            # Collect all pixels on the circle into vectoe and
            # check whether there are 12 continuous pixels darker or lighter.
            around_pixels = []
            around_pixels_cmp = []
            for pixel in pixels:
                around_pixels.append((image[row + pixel[0], col + pixel[1]]))
                if image[row + pixel[0], col + pixel[1]] >center_upper_threshold:
                    around_pixels_cmp.append(2)
                elif image[row + pixel[0], col + pixel[1]] < center_lower_threshold:
                    around_pixels_cmp.append(1)
                else:
                    around_pixels_cmp.append(0)

            node = root

            while node.result_ == -1:
                if node.children_.get(around_pixels_cmp[node.feature_]) is not None:
                    node = node.children_[around_pixels_cmp[node.feature_]]['node']
                else:
                    break
            if count % 100 == 0:
                elapsed_seconds =datetime.datetime.now() - time_for_node_traverse;
                # print(count, ": Has gone ",  elapsed_seconds.seconds ," s")
                time_for_node_traverse = datetime.datetime.now();
      
            if node.result_ == 1:
                circle += 1
                corners.append((row, col))
    
    for (row, col) in corners:
        cv2.circle(image, (col, row), 1, (0, 0, 255))

    print('Find ', circle, ' circles...')

    now = datetime.datetime.now()
    current_time = now.strftime("%H:%M:%S")
    cv2.imwrite('haha_'+ str(threshold) + '_' + str(continous_pixels) + '_'+ str(images_size) + '_' + current_time + '.jpg', image)

    # image = readAllImages('/home/yeren/Simple-SLAM/test')[0]
    # corners = []
    # for row in range(4, image.shape[0] - 4):
    #     for col in range(4, image.shape[1] - 4):
            
    #         if fast12.is_a_corner(image, col, row, threshold) != 0:
    #             corners.append((row, col))
    # for idx, corner in enumerate(corners):
    #     print(idx)
    #     cv2.circle(image, (corner[1], corner[0]), 2, (0, 0, 255), 1)
    # cv2.imwrite('haha_official.jpg', image)

            