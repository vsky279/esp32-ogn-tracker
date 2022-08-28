// Copyright 2018 Evandro Luis Copercini
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Based on Arduino libraty BluetoothSerial by Evandro Luis Copercini, modified by @voborsky

#ifndef __BTSERIAL_H__
#define __BTSERIAL_H__

typedef void (*BluetoothSerialDataCb) (const uint8_t *buffer, size_t size);
typedef void (*ConfirmRequestCb) (uint32_t num_val);
typedef void (*AuthCompleteCb) (bool success);

int BTSerial_Init(void);
int BluetoothSerial_read(void);
size_t BluetoothSerial_write(const uint8_t *buffer, size_t size);


#endif // __BTSERIAL_H__