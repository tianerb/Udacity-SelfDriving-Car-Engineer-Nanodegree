import os
import csv
import cv2
import numpy as np

### Import csv file to `allLines`

# ===== Deprecated Training Data =====
# lines = []
# with open ('./data/driving_log.csv') as csvfile:
#     reader = csv.reader(csvfile)
#     for line in reader:
#         lines.append(line)
# lines = lines[1:]
# moreTruns = []
# with open ('./moreTurns/driving_log.csv') as csvfile:
#     reader = csv.reader(csvfile)
#     for line in reader:
#         moreTurns.append(line)
# moreTurns = moreTurns[1:]
# ===== Deprecated Training Data =====

# ===== Deprecated Training Data =====

myLines = []
with open ('./myData/driving_log.csv') as csvfile:
    reader = csv.reader(csvfile)
    for line in reader:
        myLines.append(line)
myLines = myLines[1:]

moreData = []
with open ('./moreData/driving_log.csv') as csvfile:
    reader = csv.reader(csvfile)
    for line in reader:
        moreData.append(line)
moreData = moreData[1:]

allLines = myLines + moreData


# ===== Deprecated: Without Using Generator =====
# images = []
# angles = []
### Without using side cameras
# for line in allLines:
#     current_path = './allData/' + line[0].split('/')[-1]
#     image = cv2.imread(current_path)
#     images.append(image)
#     angle = float(line[3])
#     angles.append(angle)
#     # Append Flipped image and angle
#     images.append(cv2.flip(image, 1))
#     angles.append(angle * -1.0)

### Using side cameras
# for line in lines:
#     for i, adjustment in zip(range(3), (0, 0.2, -0.2)):
#         current_path = './data/IMG/' + line[i].split('/')[-1]
#         image = cv2.imread(current_path)
#         images.append(image)
#         angle = float(line[3])
#         angles.append(angle + adjustment)
#         # Append Flipped image and angle
#         images.append(cv2.flip(image, 1))
#         angles.append(angle * -1.0)

# X_train = np.array(images)
# y_train = np.array(angles)
# ===== Deprecated: Without Using Generator =====

# ===== Using Generator =====
from sklearn.model_selection import train_test_split
from sklearn.utils import shuffle

train_samples, validation_samples = train_test_split(allLines, test_size=0.2)

def generator(samples, batch_size=32, center_only = True):
    while 1: # Loop forever so the generator never terminates
        shuffle(samples)
        for offset in range(0, len(samples), batch_size):
            batch_samples = samples[offset : offset+batch_size]

            images = []
            angles = []
            if center_only:
                for line in batch_samples:                    
                    current_path = './allData/' + line[0].split('/')[-1]
                    image = cv2.imread(current_path)
                    angle = float(line[3])
                    images.append(image)
                    angles.append(angle)
                    # Append Flipped image and angle
                    images.append(cv2.flip(image, 1))
                    angles.append(angle * -1.0)
            else:
                for line in batch_samples:
                    for i, adjustment in zip(range(3), (0, 0.2, -0.2)):
                        current_path = './allData/' + line[i].split('/')[-1]
                        image = cv2.imread(current_path)
                        angle = float(line[3]) + adjustment
                        images.append(image)
                        angles.append(angle)
                        # Append Flipped image and angle
                        images.append(cv2.flip(image, 1))
                        angles.append(angle * -1.0)
            X_train = np.array(images)
            y_train = np.array(angles)
            yield shuffle(X_train, y_train)

# compile and train the model using the generator function
train_generator = generator(train_samples, batch_size=128, center_only = False)
validation_generator = generator(validation_samples, batch_size=128, center_only = False)

from keras.models import Sequential
from keras.layers import Lambda, Flatten, Dense, Cropping2D, Dropout
from keras.layers.convolutional import Convolution2D
from keras.layers.pooling import MaxPooling2D

model = Sequential()

model.add(Lambda(lambda x: (x / 127.5) - 1.0, input_shape=(160,320,3)))
model.add(Cropping2D(cropping=((70,25), (0,0))))

model.add(Convolution2D(24,5,5, subsample=(2,2), activation='relu'))
model.add(Convolution2D(36,5,5, subsample=(2,2), activation='relu'))
model.add(Convolution2D(48,5,5, subsample=(2,2), activation='relu'))
model.add(Convolution2D(64,3,3, activation='relu'))
model.add(Convolution2D(64,3,3, activation='relu'))
model.add(Flatten())
model.add(Dense(100))
model.add(Dense(50))
model.add(Dense(10))
model.add(Dense(1))

model.compile(loss='mse', optimizer='adam')

model = Sequential()

model.add(Lambda(lambda x: (x / 127.5) - 1.0, input_shape=(160,320,3)))
model.add(Cropping2D(cropping=((70,25), (0,0))))

model.add(Convolution2D(24,5,5, subsample=(2,2), activation='relu'))
model.add(Convolution2D(36,5,5, subsample=(2,2), activation='relu'))
model.add(Convolution2D(48,5,5, subsample=(2,2), activation='relu'))
model.add(Convolution2D(64,3,3, activation='relu'))
model.add(Convolution2D(64,3,3, activation='relu'))
model.add(Flatten())
model.add(Dense(100))
model.add(Dense(50))
model.add(Dense(10))
model.add(Dense(1))

model.compile(loss='mse', optimizer='adam')


from keras.models import Model
import matplotlib.pyplot as plt

### print the keys contained in the history object
print(history_object.history.keys())

### plot the training and validation loss for each epoch
plt.plot(history_object.history['loss'])
plt.plot(history_object.history['val_loss'])
plt.title('model mean squared error loss')
plt.ylabel('mean squared error loss')
plt.xlabel('epoch')
plt.legend(['training set', 'validation set'], loc='upper right')
plt.show()

