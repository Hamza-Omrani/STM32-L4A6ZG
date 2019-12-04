Issue

Line 373 :       
if(xQueueReceive(xQueue, &RxData, 0))
here we receive data, store it on a queue, then we put it again on the same buffer  which is RxData. 
the data is not stored completey if we use the regular mode which is : 
if(xQueueReceive(xQueue, &Rx_Data, 0))
where Rx_Data contain the queue item. 
Really the task takes not the time to store data to Rx_Data that's why when we use  RxData it replaces  data on the same buffer but that's not what we need here. 

So we have to correct the task periodicity!!!!

