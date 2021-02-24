import pandas as pd
import numpy as np
import csv
import time
import datetime
import matplotlib.pyplot as plt
import streamlit as st
import altair as alt
from altair import Chart
from get_dist import Lidar
import joblib
from tensorflow.keras.models import load_model

DEGREES = 150

st.title("LIDAR Test!")

@st.cache(allow_output_mutation=True, suppress_st_warning = True)
def init_lidar():
    return Lidar(DEGREES), load_model('models/lidar_v2.h5')


def classify(distances, MODEL):    
    prediction = MODEL.predict_classes(np.array([distances])[0:1])[0]
    return int(prediction)


def normalize(distances):
    distances = [val if val > 0.01 else 1 for val in distances]
    distances = [val if val < 1 else 1 for val in distances]

    return np.array(distances)


model, classifier = init_lidar()

file_name = st.sidebar.text_input("Enter Filename: ", value = "saved_distances")
file_name += ".csv"

try:
    dummy_file = pd.read_csv(file_name, header = None)
    st.write("Total Saved Rows: " + str(len(dummy_file)))
except:
    pass

ctrl = st.sidebar.selectbox('LIVE DATA', ["STOP", "START"])
collect = st.sidebar.selectbox("COLLECT DATA", ["NO", "YES"])


def get_new_values(model, classifier, collect, file_name):

    classes = {
        0: "Left",
        1: "Forward",
        2: "Right",
        3: "STOP!"
    }

    global SAVED_DATA

    distances = model.get_distance()
    distances = normalize(distances)
    cls_val = classes[classify(distances, classifier)]

    if collect == "YES":
        with open(file_name, 'a', newline='') as out_file:
            writer = csv.writer(out_file)
            writer.writerow(distances)
        out_file.close()

    distances = [(val if val <= 1.0 else 1.0) for val in distances]
    df = pd.DataFrame(np.array([list(range(DEGREES)), distances]).T, columns=['degrees', 'distance'])
    scatter = alt.Chart(df).mark_circle(size = 50).encode(x = "degrees", y = "distance", color=alt.Color('distance', legend=alt.Legend(title="Class: " + cls_val))).properties(width = 650, height = 400)
    return scatter


df = pd.DataFrame(np.array([list(range(DEGREES)), np.random.randn(DEGREES)]).T, columns = ['degrees', 'distance'])
scatter = alt.Chart(df).mark_circle(size = 50).encode(x = "degrees", y = "distance", color=alt.Color('distance', legend=alt.Legend(title="Class: NULL"))).properties(width = 650, height = 400)
my_chart = st.altair_chart(scatter)


if ctrl == ("START"):
    while True:
        scatter = get_new_values(model, classifier, collect, file_name)
        my_chart.altair_chart(scatter)