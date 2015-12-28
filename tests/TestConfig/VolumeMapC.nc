#include "StorageVolumes.h" 

configuration VolumeMapC { 
  provides { 
    interface BlockRead[uint8_t volume_id]; 
    interface BlockWrite[uint8_t volume_id]; 
    interface LogRead[uint8_t volumeId]; 
    interface LogWrite[uint8_t volumeId]; 
    interface Mount[uint8_t volumeId]; 
    interface ConfigStorage[uint8_t volumeId]; 
  } 
} 

implementation { 
  components VolumeMapP; 

  BlockRead = VolumeMapP; 
  BlockWrite = VolumeMapP; 
  LogRead = VolumeMapP; 
  LogWrite = VolumeMapP; 
  Mount = VolumeMapP; 
  ConfigStorage = VolumeMapP; 
} 
