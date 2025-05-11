from flask import Flask, render_template, jsonify

app = Flask(__name__)

@app.route("/")
def index():
    return render_template("index.html")

@app.route("/status")
def status():
    return jsonify({"power": "ON", "depth": 1.8, "obstacle_distance": 40})

if __name__ == "__main__":
    app.run(debug=True)
