#!/usr/bin/env python

from sklearn.externals import joblib

class SVMObjectClassifier:
    """
    Defines an SVM classifier with the mean and standard deviation of
    the features, and a label encoder

    """
    def __init__(self, classifier, label_encoder, mean, std):
        self.classifier = classifier
        self.label_encoder = label_encoder
        self.mean = mean
        self.std = std

    def classify(self, feature_vector):
        feature_vector -= self.mean
        feature_vector /= self.std
        prediction = self.classifier.predict(feature_vector)
        return self.label_encoder.inverse_transform(prediction)[0]
    
    @classmethod
    def load(cls, classifier_name, label_encoder_name):
        classifier = joblib.load(classifier_name)
        [label_encoder, mean, std] = joblib.load(label_encoder_name)
        return SVMObjectClassifier(classifier, label_encoder, mean, std)
