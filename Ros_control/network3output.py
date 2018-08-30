import numpy as np
import xlrd
from keras.models import model_from_json
from keras.models import load_model
from keras.models import Sequential
from keras.layers.core import Dense, Dropout, Activation
from keras.layers import Conv2D, MaxPooling2D, Flatten
from keras.optimizers import SGD, Adam
from keras.utils import np_utils
from keras.datasets import mnist

train = xlrd.open_workbook('data1.xls')
test = xlrd.open_workbook('test3.xls')
train_sheet = train.sheets()[0]
test_sheet = test.sheets()[0]
# print(train_sheet.row_values(0))

train_data_x = []
train_data_y = []
test_data_x = []
test_data_y = []
# print(train_sheet.nrows)
for i in range(1, train_sheet.nrows):
# for i in range(1, 1000):
    rows = train_sheet.row_values(i)
    train_data_x.append(rows[0:3])
    train_data_y.append(rows[3:6])
    # print(rows[0:3])
    # print(rows[3:])
    # exit(0)
train_sheet1 = xlrd.open_workbook('data2.xls').sheets()[0]
for i in range(1, train_sheet1.nrows):
# for i in range(1, 1000):
    rows = train_sheet.row_values(i)
    train_data_x.append(rows[0:3])
    train_data_y.append(rows[3:6])
# for i in range(1800, 2000):
# for i in range(1, 1000):
#     rows = train_sheet.row_values(i)
#     test_data_x.append(rows[0:3])
#     test_data_y.append(rows[3:])


for i in range(1, test_sheet.nrows):
    rows = test_sheet.row_values(i)
    test_data_x.append(rows[0:3])
    test_data_y.append(rows[3:6])

# train_data_x = np.array(train_data_x)
# train_data_y = np.array(train_data_y)
# test_data_x = np.array(test_data_x)
# test_data_y = np.array(test_data_y)

data_round = 6
train_data_x = np.round(np.array(train_data_x), data_round)
train_data_y = np.round(np.array(train_data_y), data_round)
test_data_x = np.round(np.array(test_data_x), data_round)
test_data_y = np.round(np.array(test_data_y), data_round)
print train_data_x.shape, test_data_y.shape

model = Sequential()
# input layer
model.add(Dense(input_dim=3, units=100, activation='tanh'))
# hidden layer
model.add(Dense(units=100, activation='tanh'))
model.add(Dense(units=100, activation='tanh'))
model.add(Dense(units=100, activation='tanh'))
# output layer
# model.add(Dropout(0.5))
model.add(Dense(units=3, activation='tanh'))  # maybe 3

model.compile(loss='mse', optimizer=SGD(lr=0.1), metrics=['accuracy'])
# model.compile(loss='mse', optimizer='adam')
model.fit(train_data_x, train_data_y, batch_size=50, epochs=70)

train_result = model.evaluate(train_data_x, train_data_y)
test_result = model.evaluate(test_data_x, test_data_y)
print 'Train ACC = ', train_result[1]
print 'Test ACC = ', test_result[1]
model.save('model3output/'+str(round(test_result[1], 3))+'.h5')
