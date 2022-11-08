#!/bin/sh
export FLASK_APP=./scripts/rest_client.py
pipenv run flask --debug run -h 0.0.0.0