#!/usr/bin/env python

import pickle
import itertools
import numpy as np
import matplotlib.pyplot as plt
from sklearn.svm import LinearSVC 
from sklearn.preprocessing import LabelEncoder, StandardScaler
from sklearn.model_selection import StratifiedKFold, cross_val_score, cross_val_predict
from sklearn import metrics


def plot_confusion_matrix(cm, classes, normalize=False, 
                          title='Confusion matrix', cmap=plt.cm.Blues):
    """Print and plot the confusion matrix.

    Normalization can be applied by setting `normalize=True`.
    """
    plt.imshow(cm, interpolation='nearest', cmap=cmap)
    plt.title(title, fontsize=16)
    cbar = plt.colorbar()
    cbar.ax.tick_params(labelsize=16)
    tick_marks = np.arange(len(classes))
    plt.xticks(tick_marks, classes, rotation=90)
    plt.yticks(tick_marks, classes)

    if normalize:
        cm = cm.astype('float') / cm.sum(axis=1)[:, np.newaxis]

    thresh = cm.max() / 2.
    for i, j in itertools.product(range(cm.shape[0]), range(cm.shape[1])):
        plt.text(j, i, '{0:.1f}'.format(cm[i, j]), horizontalalignment="center",
                 color="white" if cm[i, j] > thresh else "black", fontsize=16)

    plt.tight_layout()
    plt.tick_params(labelsize=18)
    plt.ylabel('True label', fontsize=20)
    plt.xlabel('Predicted label', fontsize=20)


training_set = pickle.load(open('training_set.pkl', 'rb'))

feature_list = []
label_list = []

for item in training_set:
    if np.isnan(item[0]).sum() < 1:
        feature_list.append(item[0])
        label_list.append(item[1])

print('Features in Training Set: {}'.format(len(training_set)))
print('Invalid Features in Training set: {}'.
      format(len(training_set) - len(feature_list)))

X = np.array(feature_list)
X_scaler = StandardScaler().fit(X)
X_train = X_scaler.transform(X)
y_train = np.array(label_list)

# Convert label strings to numerical encoding
encoder = LabelEncoder()
y_train = encoder.fit_transform(y_train)

clf = LinearSVC(C=0.001)

splitter = StratifiedKFold(n_splits=10, shuffle=True, random_state=42)
cv = splitter.split(X_train, y_train)

scores = cross_val_score(cv=cv, estimator=clf, X=X_train, y=y_train, 
                         scoring='accuracy')
print('Scores: ' + str(scores))
print('Accuracy: %0.2f (+/- %0.2f)' % (scores.mean(), 2*scores.std()))

# The function cross_val_predict has a similar interface to 
# cross_val_score, but returns, for each element in the input, 
# the prediction that was obtained for that element when it 
# was in the test set. Only cross-validation strategies that 
# assign all elements to a test set exactly once can be used 
# (otherwise, an exception is raised). For example, the 
# splitter cannot be StratifiedShuffleSplitter().
predictions = cross_val_predict(cv=splitter, estimator=clf, 
                                X=X_train, y=y_train)

accuracy_score = metrics.accuracy_score(y_train, predictions)
print('accuracy score: ' + str(accuracy_score))

confusion_matrix = metrics.confusion_matrix(y_train, predictions)

class_names = encoder.classes_.tolist()

clf.fit(X=X_train, y=y_train)

model = {'classifier': clf, 'classes': encoder.classes_, 'scaler': X_scaler}

pickle.dump(model, open('model.pkl', 'wb'))

plt.figure()
plot_confusion_matrix(confusion_matrix, classes=encoder.classes_,
                      title='Confusion matrix, without normalization')

plt.figure()
plot_confusion_matrix(confusion_matrix, classes=encoder.classes_, 
                      normalize=True, title='Normalized confusion matrix')

plt.show()

