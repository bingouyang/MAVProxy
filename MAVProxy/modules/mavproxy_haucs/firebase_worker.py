from queue import Queue, Empty
import os
import firebase_admin
from firebase_admin import credentials, db
import concurrent.futures
from PyQt5.QtCore import QThread, pyqtSignal, QObject, QMutex, QMutexLocker
import pandas as pd
from datetime import datetime
import numpy as np
import shutil
import pickle
import logging

#init logger
logger = logging.getLogger(__name__)

def convert_numpy(obj):
    if isinstance(obj, np.generic):
        return obj.item()
    elif isinstance(obj, (np.ndarray, list, tuple)):
        return [convert_numpy(o) for o in obj]
    elif isinstance(obj, dict):
        return {k: convert_numpy(v) for k, v in obj.items()}
    else:
        return obj

class FirebaseWorker(QThread):

    app = None
    cred = None
    fail_counter = 0

    max_fail = 30
    fb_key="fb_key.json"
    database_folder = "database_truck"
    unsaved_folder = "unsaved"
    completed_folder = "completed"
    sdatas = []

    def __init__(self, database_mutex):
        super().__init__()
        self._abort = False
        logger.info("starting firebase worker")
        self.database_mutex = database_mutex
        self.init_firebase() #TODO unecessary function

    def init_firebase(self):
        try:
            if os.path.exists(self.fb_key):
                self.cred = credentials.Certificate(self.fb_key)
                self.app = firebase_admin.initialize_app(self.cred, {'databaseURL': 'https://haucs-monitoring-default-rtdb.firebaseio.com'})
            else:
                logger.error('firebase initialization failed bc no fb_key found')
        except Exception as error:
            logger.error(f'Firebase initialize failed {str(error)}')

    def restart_firebase(self, in_app):
        logger.info("attempting to restart firebase")
        if in_app is not None:
            firebase_admin.delete_app(in_app)
            self.msleep(10000)
        if os.path.exists(self.fb_key) and self.cred is None:
            self.cred = credentials.Certificate(self.fb_key)
        if self.cred is not None:
            new_app = firebase_admin.initialize_app(self.cred,
                                                {'databaseURL': 'https://haucs-monitoring-default-rtdb.firebaseio.com'})
            return new_app
        return None

    def add_sdata(self, sdata, row):
        today_str = datetime.now().strftime("%Y-%m-%d")
        folder = self.database_folder
        os.makedirs(folder, exist_ok=True)
        file_path = os.path.join(folder, f"iamtruck_{today_str}.csv")
        # lock access to database folder
        with QMutexLocker(self.database_mutex):
            try:
                # random csv stuff
                if os.path.exists(file_path):
                    df = pd.read_csv(file_path)
                    df = pd.concat([df, pd.DataFrame([row])], ignore_index=True)
                else:
                    df = pd.DataFrame([row]) 
                df.to_csv(file_path, index=False)
            except Exception as e:
                logger.info("could not append to local database file %s", e)

        self.save_data_pickle(sdata)

    def save_data_pickle(self, sdata):
        message_time = sdata['message_time']
        save_time = message_time.replace(":","-")
        file_path = os.path.join(self.unsaved_folder, save_time + ".pickle")
        self.sdatas.append(sdata)

        os.makedirs(self.unsaved_folder, exist_ok=True)
        with open(file_path, 'wb') as file:
            pickle.dump(sdata, file)
        
        logger.info(f"saved pickle: {file_path}")

    def move_pickle_to_completed(self, sdata):
        save_time = sdata['message_time'].replace(":", "-")
        src_path = os.path.join(self.unsaved_folder, save_time + ".pickle")
        dst_path = os.path.join(self.completed_folder, save_time + ".pickle")

        try:
            os.makedirs(self.completed_folder, exist_ok=True)
            shutil.move(src_path, dst_path)
            logger.info(f"Moved pickle file to completed: {dst_path}")
        except FileNotFoundError:
            logger.warning(f"Source file not found, saving new pickle: {dst_path}")
        except Exception as e:
            logger.error(f"Failed to move pickle: {src_path} → {dst_path} — {e}")
    
    def run(self):
        folder = self.unsaved_folder
        if not os.path.exists(folder):
            return

        files = [f for f in os.listdir(folder) if f.endswith(".pickle")]
        for filename in files:

            file_path = os.path.join(folder, filename)
            with open(file_path, 'rb') as file:
                sdata = pickle.load(file)

            self.sdatas.append(sdata)

        while not self._abort:
            self.update_firebase_when_internet()
            if self._abort:
                break
            self.msleep(2000)

    def abort(self):
        self._abort = True

    def update_firebase_when_internet(self):
        for i in reversed(range(len(self.sdatas))):
            sdata = self.sdatas[i]

            upload_status = self.update_firebase(sdata)
            if upload_status:
  
                self.move_pickle_to_completed(sdata)

                logger.info('data upload to firebase complete')
                del self.sdatas[i]

                # lock access to database folder
                with QMutexLocker(self.database_mutex):
                    try:
                        msg_time_str = sdata['message_time']
                        msg_date = datetime.strptime(msg_time_str.split("_")[0], "%Y%m%d").strftime("%Y-%m-%d")
                        file_path = os.path.join(self.database_folder, f"iamtruck_{msg_date}.csv")
                        if os.path.exists(file_path):
                            df = pd.read_csv(file_path)

                            match_index = df[df["message_time"] == msg_time_str].index

                            if not match_index.empty:
                                df.loc[match_index, "upload status"] = True
                                df.to_csv(file_path, index=False)
                        
                    except Exception as e:
                        logger.warning('failed to update csv')

    def update_firebase(self, sdata):        
        # copy relevant information to firebase
        upload_data = {}
        upload_data['do'] = sdata['do_vals']
        upload_data['ysi_do_mgl'] = sdata['ysi_do_mgl_arr']
        upload_data['heading'] = sdata['hdg']
        upload_data['init_do'] = 1 #hardcoded to handle legacy website
        upload_data['init_pressure'] = sdata['init_pressure']
        upload_data['lat'] = sdata['lat']
        upload_data['lng'] = sdata['lng']
        upload_data['pid'] = sdata['pid']
        upload_data['pressure'] = sdata['pressure_vals']
        upload_data['temp'] = sdata['temp_vals']
        upload_data['sid'] = sdata['name']
        upload_data['type'] = 'rpi_truck' #hardcoded truck type
        upload_data['sample_hz'] = sdata['sample_hz']
        upload_data['sensor_battv'] = sdata['battv']
        upload_data = clean_for_firebase(upload_data)

        try:
            if self.app is not None:
                db.reference('LH_Farm/pond_' + sdata['pid'] + '/' + sdata['message_time'] + '/').set(upload_data)
            else:
                return False
        except Exception as error:
            logger.warning(f"uploading data to firebase failed: {error}")
            self.fail_counter +=1
            if self.fail_counter >= self.max_fail:
                self.app = self.restart_firebase(self.app)
                self.fail_counter = 0
            return False
        return True
    
def clean_for_firebase(data):
    for key in data:
        val = data[key]
        if isinstance(val, np.ndarray):
            val = val.tolist()
    return data