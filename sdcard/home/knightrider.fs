CR .( knightrider.fs loading ... )

3 3 dmod   \ set D3 to Output
3 6 dmod   \ set D6 to Output

: left ( -- ) 
  7 0 do
    1 i dpin! 
    0 apin@ 10 / osDelay drop  \ delay depends on A0
    0 i dpin!
  loop 
;

: right ( -- )
  8 1 do  
    1 8 i - dpin! 
    0 apin@ 10 / osDelay drop  \ delay depends on A0
    0 8 i - dpin!
  loop 
;

: knigthrider ( -- )
  begin 
    left right 
    switch1? 
  until 
  0 0 dpin!
;

: knightrider-thread ( -- )
  osNewDataStack
  knigthrider
  osThreadExit
;

' knightrider-thread 0 0 osThreadNew
CR .( thread started with ID ) .

