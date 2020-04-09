from flask import Flask, jsonify, request
from flask_restful import Api, Resource
app = Flask(__name__)
api = Api(app)

class Hello(Resource):
    def get(self):
        return "hello"

api.add_resource(Hello, '/hello')

if __name__ == "__main__":
    app.run(host="127.0.0.1", debug=True)
	print("Running!!!!!")
