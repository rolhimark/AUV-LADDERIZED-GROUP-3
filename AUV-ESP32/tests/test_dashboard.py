import requests

def test_status_endpoint():
    r = requests.get("http://localhost:5000/status")
    assert r.status_code == 200
