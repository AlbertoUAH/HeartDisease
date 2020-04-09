from flask import Flask, jsonify, request
from flask_restful import Api, Resource
from sklearn.model_selection import train_test_split
from sklearn.tree import DecisionTreeClassifier
import pandas as pd

app = Flask(__name__)
api = Api(app)

class Hello(Resource):
    def get(self):
        """df = pd.read_csv("data.csv", sep = ";")
        df = df.dropna()

        X = df.drop('diagnosis', axis=1)
        y = df['diagnosis']
        X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.15)

        classifier = DecisionTreeClassifier()
        classifier.fit(X_train, y_train)

        y_pred = classifier.predict(X_test)

        id_list = list(X_test['id'])
        diagnosis_list = []

        for index, row in df.iterrows():
            if row['id'] in id_list:
                diagnosis_list.append(row['diagnosis'])"""
        return "hello"

api.add_resource(Hello, '/hello')

if __name__ == "__main__":
    app.run(host="127.0.0.1", debug=True)
	print("Running!!!!!")
