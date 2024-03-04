; Program description:
;   The program below creates a visualization of a snake-like entity
;   moving on the terminal. The terimnal is set to 25 lines and 80 columns.
;   On the screen there is "road" consisting of 'A's (lines 10 & 16).
;   A "snake" made out of 'B's starts from the beginning of line 13 
;   and progresses one pixel forward every 100ms. If it reaches the end 
;   of line 13, then it loops back to its beginning. Also, a box that
;   alternates between green and red each second is printed on the
;   upper right corner of the terminal. When the box turns red, the 
;   snake's size is extended by an integer amount between [1, 4]. 
;   When the snake takes up the entire line in which it moves on, 
;   then the program finishes with an exit message on the screen.
;   
;
; Author: Yfantidis Dimitrios
; Creation Date: 25/12/2023
; Revisions:
;   - 25/12/2023: Configured video memory (in code). 
;                 Implemented the Road of 'A's (BUILD_ROAD routine) and the pulsating box (SHOW_BOX routine).
;   - 26/12/2023: Introduced the snake of 'B's by adding the DRAW_SNAKE routine.
;   - 28/12/2023: Implemented the Pseudorandom number generator in the GENERATOR routine.
;   - 29/12/2023: Sped up the snake's movement with the routine MOVE_SNAKE. 
;                 Sped up the snake's extension by replacing DRAW_SNAKE with DRAW_SNAKE_OPT.
;                 Removed DRAW_SNAKE.
;   - 06/01/2023: Extended comments
; Date: 06/12/2023


.model flat, stdcall
.stack 1024

.data
generatorSeed   DW  ?       ; The pseudo-random number generator's seed (depends on system time)
snakeLength     DW  1       ; The length of the snake of in characters. It is increased randomly.
snakeHeadPos    DW  12*2*80 ; The position of the snake's head in the terminal (starts from the 1st column of the 13th line)

; The message that will be printed on the terminal after the program finishes (according to the assignment's prompt)
exit_message    DB  "It's not a bug; it's an undocumented feature", 0
; The string of the road's edges
road_string     DB  "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA$"



.code
_start:
    ; Sets terminal's dimension to 80x25 16 color text (CGA, EGA, MCGA, VGA)
    MOV     AX, 0003h
    INT     10h
    
    MOV     AH, 00h  ; interrupts to get system time        
    INT     1Ah      ; CX:DX now hold number of clock ticks since midnight
    
    ; Load data segment
    MOV     AX, @data
    MOV     DS, AX
    
    ; Set the LSBs of ticks since midnight as the generator's seed
    MOV     [generatorSeed], DX
    
    ; Road constructed only once
    CALL    BUILD_ROAD
    
    ; Alternates between 1 (green box) and 0 (red box)
    MOV     AX, 1
    
    ; The generator's result will be divided with 4 to map 
    ; the value from [0, 2^16-1] to [0, 3]
    MOV     BX, 4
    
main_loop:
    ; Produces ZF = 1 if AL = 0, else ZF = 0
    OR      AL, AL
    JZ      else
    ; If AL = 1 then the box is green
    ; Push the green color
    PUSH    0AA00h
    ; Prints box (callee clears the stack)
    CALL    SHOW_BOX
    JMP     endif    
else:
    ; if AL = 0 then the box is red
    ; Extend the snake's length by (rand() % 4) + 1
    PUSH    AX
    
    MOV     AX, [generatorSeed] ; Load previous value to AX
    CALL    GENERATOR           ; Fetch the next random integer
    MOV     [generatorSeed], AX ; Update the previous value
    
    XOR     DX, DX  ; Set DX:AX = 0000:rand()
    DIV     BX      ; Fetch modulo at DX
    INC     DX      ; Map DX from [0, 3] to [1, 4]
    
    ; Push the red color
    PUSH    0CC00h
    ; Prints box
    CALL    SHOW_BOX
    
    ; Push function arguments (snake's position, snake's lenght and N-extension of the snake's tail)
    PUSH    [snakeHeadPos]
    PUSH    [snakeLength]
    PUSH    DX
    ; Extend the snake
    CALL    DRAW_SNAKE_OPT
    
    ADD     [snakeLength], DX   ; Extend the snake's tail
    ; Fetch the color condition of the box (AX=0 red, AX=1 green)
    POP     AX
    
    ; Break loop if the sanke takes up the entire line (80 pixels width)
    CMP     [snakeLength], 80
    JAE     exit_label
endif:
    ; Negate AX's (boolean) value to change color
    XOR     AL, 1
    
    ; Move the snake for 10 iterations
    MOV     CX, 10
    ; Used inside the snake_loop
    MOV     DX, 86A0h
    
snake_loop:
    ; Save the iteration counter (CX will be manipulated)
    PUSH    CX
    
    ; Now CX:DX = 186A0h = 100000 microseconds = 100 milliseconds
    MOV     CX, 0001h
    ; Wait Interrupt: Halt execution for 100ms
    MOV     AH, 86h
    INT     15h
    
    ; Move the snake
    PUSH    [snakeHeadPos]
    PUSH    [snakeLength]
    CALL    MOVE_SNAKE
    
    ; Move snake's head one pixel right
    MOV     DX, 2
    ADD     [snakeHeadPos], DX
    
    CMP     [snakeHeadPos], 13*2*80
    JNE     endif_head_check
    ; Doesn't allow the snake to move to the next line
    SUB     [snakeHeadPos], 160
    
endif_head_check:
    ; Fetch the iteration counter
    POP     CX
    LOOP    snake_loop
    ; Each iteration of the inner loop halts the program for 100ms.
    ; Thus, each iteration of the outer loop happens every ~1s meaning 
    ; that the box pulsates every ~1s.
    JMP     main_loop
    
exit_label:
    ; Draws the snake one last time (since its length is now >=80)
    MOV     CX, 80
    MOV     BX, 12*2*80
    
    MOV     AX, 0B800h
    MOV     DS, AX
    
    MOV     AL, 'B'
    MOV     AH, 0Fh
    ; Fill line 13 with 'B's
fill_snake:
    MOV     [BX], AX
    ADD     BX, 2
    LOOP    fill_snake
    
    
    LEA     BP, exit_message
    
    ; Push the text string's address and it's start position on the terminal    
    PUSH    BP
    PUSH    19*2*80+40
    CALL    PRINT_MESSAGE

	MOV     AH, 4Ch ; DOS function: Exit program   
	INT     21h     ; DOS interrupt


; Fills line 10 and 16 with 'A's
; Parameters: None
; Returns: Nothing
PROC    BUILD_ROAD near
    ; Save registers that will be used
    PUSH    AX
    PUSH    BX
    PUSH    CX
    PUSH    DX  
    
    ; Set cursor position to 10th line and 1st column
    MOV     AH, 02h ; Function number for setting cursor position
    MOV     BH, 0   ; Page number
    MOV     DH, 9   ; Row (Y coordinate)
    MOV     DL, 0   ; Column (X coordinate)
    INT     10h     ; Call video interrupt

    ; Print side of road
    MOV     AH, 09h                 ; Function number for displaying string
    MOV     DX, offset road_string  ; DS:DX points to the string
    INT     21h                     ; Call DOS interrupt   
  
    ; Set cursor position to 16th line and 1st column
    MOV     AH, 02h ; Function number for setting cursor position
    MOV     BH, 0   ; Page number
    MOV     DH, 15  ; Row (Y coordinate)
    MOV     DL, 0   ; Column (X coordinate)
    INT     10h     ; Call video interrupt

    ; Print side of road
    MOV     AH, 09h                 ; Function number for displaying string
    MOV     DX, offset road_string  ; DS:DX points to the string
    INT     21h                     ; Call DOS interrupt   
   
    ; Restore used Registers
    POP     DX
    POP     CX  
    POP     BX
    POP     AX
    RET 
ENDP    BUILD_ROAD



; Draws a box on the upper right corner of the terminal.
; It is actually 6x4 colored null characters, but the characters
; have a greater height than width, thus 6x4 seems more like a box 
; instead of 4x4 or 6x6.
; Parameters:
;   - arg1: Character that makes up the box and its attribute byte
; Returns: Nothing
PROC    SHOW_BOX near
    ; Prologue
    PUSH    BP
    MOV     BP, SP
    
    ; Save
    PUSH    AX
    PUSH    BX
    PUSH    CX
    PUSH    DS
    
    ; Access Video Memory
    MOV     AX, 0B800h
    MOV     DS, AX
    
    ; Load the function argument to AX
    MOV     AX, [BP+4]
    ; Position of the upper left corner of the box
    MOV     BX, 2*80+2*72
    ; Draw on 6 columns and 4 lines
    MOV     CX, 6
box_loop:
    MOV     [BX], AX
    MOV     [BX+1*160], AX
    MOV     [BX+2*160], AX
    MOV     [BX+3*160], AX
    ; Progress to the next column
    ADD     BX, 2
    LOOP    box_loop
    
    ; Restore
    POP     DS
    POP     CX
    POP     BX  
    POP     AX
    
    ; Epilogue
    POP     BP
    RET     2
ENDP    SHOW_BOX



; Extends the already existing snake by N 'B's
; (instead of drawing again the entire snake like DRAW_SNAKE did)
; Parameters:
;   - arg1: N extension of snake
;   - arg2: Snake's length
;   - arg3: Snake's head position on the screen
PROC    DRAW_SNAKE_OPT near
    ; Prologue
    PUSH    BP
    MOV     BP, SP
    
    ; Save
    PUSH    AX
    PUSH    BX
    PUSH    CX
    PUSH    DX
    PUSH    DS
    
    ; Load the video memory
    MOV     AX, 0B800h
    MOV     DS, AX
    
    MOV     CX, [BP+4]  ; N extension of snake
    MOV     DX, [BP+6]  ; SnakeLength
    INC     DX
    SHL     DX, 1       ; Adjustment: pixel_size = 2 and thus snake_length must be doubled
    MOV     BX, [BP+8]  ; SnakeHeadPos
    SUB     BX, DX      ; Set BX to the tip of the snake's tail
    
    MOV     AL, 'B'     ; Snake consists of 'B's
    MOV     AH, 0Fh     
    
    CMP     BX, 12*2*80-2
    JA      extend_snake
    ; If the snake's tail extends to the previous line, then make it extend at the end of the current line (line 13)
    ADD     BX, 160
extend_snake:
    ; Append a 'B' at the end of the snake
    MOV     [BX], AX
    ; Go one position back
    SUB     BX, 2
    CMP     BX, 12*2*80-2
    JA     endif_adjust_extend
    ; Append 'B's at the end of the current line instead of the previous
    ADD     BX, 160
endif_adjust_extend:
    CMP     BX, 13*2*80
    LOOP    extend_snake
    
    ; Restore
    POP     DS
    POP     DX
    POP     CX
    POP     BX
    POP     AX
    
    ; Epilogue
    POP     BP
    RET     6
ENDP    DRAW_SNAKE_OPT



; Moves the snake by one pixel
; Parameters:
;   - arg1: Snake's length
;   - arg2: Snake head's position on the screen
; Returns: Nothing
PROC    MOVE_SNAKE near
    PUSH    BP
    MOV     BP, SP
    
    ; Save registers that will be used
    PUSH    AX
    PUSH    CX
    PUSH    BX
    PUSH    DS
    
    ; Load the video memory
    MOV     AX, 0B800h
    MOV     DS, AX
    
    MOV     CX, [BP+4]  ; SnakeLength
    SHL     CX, 1
    MOV     BX, [BP+6]  ; SnakeHeadPos
    
    MOV     AL, 'B'     ; Snake consists of 'B's
    MOV     AH, 0Fh     ; Black foreground, white character
    
    ; Append the next 'B'
    MOV     [BX], AX
    ; Point BX to the last 'B' on the screen
    SUB     BX, CX
    CMP     BX, 12*2*80-2
    JA      endif_adjust_bx
    ; Adjust screen position if BX moved to the previous line (line 12)
    ADD     BX, 160
endif_adjust_bx:
    ; Set attribute byte to pitch black
    XOR     AH, AH
    ; Clear the last 'B' to move the snake by writting a black 'B' on it
    MOV     [BX], AX
    
    ; Restore the used registers
    POP     DS
    POP     BX
    POP     CX  
    POP     AX    
       
    POP     BP
    RET     4
ENDP    MOVE_SNAKE    



; Simple 16-bit Linear Feedback Shift Register (LFSR): Generates a random unsigned WORD.
; The next value is generated by right shifting the previous to the right once 
; and appending as MSB the XOR of the 6th, 8th and 13th bit of the previous value.
; Parameters:
;   - AX: the previous number that was generated (or the seed for the first time)
; Returns: the next number of the generator
PROC    GENERATOR near              
    ; Save
    PUSH    BX
    PUSH    DX
    
    ; Simple LFSR implementation
    XOR     BX, BX
    
    MOV     DX, AX
    SHR     DX, 5
    AND     DX, 1
    XOR     BX, DX
    
    MOV     DX, AX
    SHR     DX, 7
    AND     DX, 1
    XOR     BX, DX
    
    MOV     DX, AX
    SHR     DX, 12
    AND     DX, 1
    XOR     BX, DX
    
    ROR     BX, 1
    SHR     AX, 1
    OR      AX, BX
  
    
    ; Restore           
    POP     DX
    POP     BX
    
    RET     
ENDP    GENERATOR


; Prints an alphanumeric string on the terminal.
; Parameters:
;   - arg1: The position of the first character on the screen.
;   - arg2: The alphanumeric string's address.
; Returns: Nothing
PROC    PRINT_MESSAGE near
    ; Prologue
    PUSH    BP
    MOV     BP, SP
    
    ; Save
    PUSH    AX
    PUSH    BX
    PUSH    DX
    PUSH    DS
    PUSH    DI
    PUSH    SI
    
    MOV     DI, [BP+4]  ; Position on screen to print
    MOV     SI, [BP+6]  ; String's address  
    
    MOV     BH, 0Fh     ; White on black atribute byte
    
    MOV     AX, @data
    MOV     DX, 0B800h
    MOV     DS, AX

print_loop:
    MOV     BL, [SI]    ; Fetch the next character from the data segment
    MOV     DS, DX
    MOV     [DI], BX    ; Print the character to the next position in the video memory
    ADD     DI, 2       ; Point to the next pixel
    INC     SI          ; Point to the next character
    MOV     DS, AX
    CMP     [SI], 0     ; Check for null terminating character
    JNE     print_loop     
    
    ; Restore
    POP     SI
    POP     DI
    POP     DS
    POP     DX
    POP     BX
    POP     AX
    
    ; Epilogue
    POP     BP
    RET     4
ENDP    PRINT_MESSAGE