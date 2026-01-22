void init_watchdog(){  
  esp_task_wdt_init(&twdt_config);     //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL);                   //add current thread to WDT watch  
}