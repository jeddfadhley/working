library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use work.common_pack.all;

entity cmdProc is
  port (
    clk         : in  STD_LOGIC;
    reset       : in  STD_LOGIC;
    rxnow       : in  STD_LOGIC;
    rxData      : in  STD_LOGIC_VECTOR (7 downto 0);
    rxdone      : out STD_LOGIC;
    ovErr       : in  STD_LOGIC;
    framErr     : in  STD_LOGIC;
    txData      : out STD_LOGIC_VECTOR (7 downto 0);
    txnow       : out STD_LOGIC;
    txdone      : in  STD_LOGIC;
    start       : out STD_LOGIC;
    numWords_bcd: out BCD_ARRAY_TYPE(2 downto 0);
    dataReady   : in  STD_LOGIC;
    byte        : in  STD_LOGIC_VECTOR (7 downto 0);
    maxIndex    : in  BCD_ARRAY_TYPE(2 downto 0);
    dataResults : in  CHAR_ARRAY_TYPE(0 to RESULT_BYTE_NUM-1);
    seqDone     : in  STD_LOGIC
  );
end cmdProc;

architecture FSM of cmdProc is

    type state_type is (
        IDLE,
        ECHO,
        ECHO_WAIT,
        GET_DIGIT,
        -- 'a' command: data processing
        START_DP,
        WAIT_DATA,
        DATA_TX,
        DATA_WAIT,
        REQ_DATA,
        -- Newline (LF + CR) before p/l output
        SEND_LF,
        SEND_LF_WAIT,
        SEND_CR,
        SEND_CR_WAIT,
        -- 'l' command: print 7 result bytes
        RES_PREP,
        RES_TX,
        RES_WAIT,
        -- 'p' command: print peak byte + index
        PEAK_PREP,
        PEAK_TX,
        PEAK_WAIT,
        PIDX_TX,
        PIDX_WAIT
    );

    type cmd_type_t is (CMD_NONE, CMD_A, CMD_P, CMD_L);

    signal state, next_state : state_type := IDLE;

    -- Command type register
    signal cmd_type, next_cmd_type : cmd_type_t := CMD_NONE;

    -- Echo register
    signal echo_reg, next_echo_reg       : std_logic_vector(7 downto 0) := (others => '0');

    -- BCD digit collection
    signal hundreds, next_hundreds       : unsigned(3 downto 0) := (others => '0');
    signal tens, next_tens               : unsigned(3 downto 0) := (others => '0');
    signal ones, next_ones               : unsigned(3 downto 0) := (others => '0');
    signal digit_count, next_digit_count : unsigned(1 downto 0) := (others => '0');

    -- Registered numWords output
    signal reg_numWords : BCD_ARRAY_TYPE(2 downto 0) := (others => "1001");

    -- TX byte register (holds current byte being transmitted as hex)
    signal tx_byte, next_tx_byte         : std_logic_vector(7 downto 0) := (others => '0');

    -- Nibble counter: 0=hi nibble, 1=lo nibble, 2=space
    signal nibble_cnt, next_nibble_cnt   : unsigned(1 downto 0) := (others => '0');

    -- Result byte index (for 'l' command, 0-6)
    signal byte_idx, next_byte_idx       : unsigned(2 downto 0) := (others => '0');

    -- Peak index digit counter (for 'p' command, 0-2)
    signal peak_idx, next_peak_idx       : unsigned(1 downto 0) := (others => '0');

    -- Latched results (captured on seqDone)
    signal reg_dataResults : CHAR_ARRAY_TYPE(0 to RESULT_BYTE_NUM-1) := (others => (others => '0'));
    signal reg_maxIndex    : BCD_ARRAY_TYPE(2 downto 0) := (others => (others => '0'));

    -- Sticky flag: set when seqDone fires, cleared on START_DP
    signal seq_done_flag : std_logic := '0';


    function to_hex(nibble : std_logic_vector(3 downto 0)) return std_logic_vector is
    begin
        case nibble is
            when "0000" => return x"30";
            when "0001" => return x"31";
            when "0010" => return x"32";
            when "0011" => return x"33";
            when "0100" => return x"34";
            when "0101" => return x"35";
            when "0110" => return x"36";
            when "0111" => return x"37";
            when "1000" => return x"38";
            when "1001" => return x"39";
            when "1010" => return x"41";
            when "1011" => return x"42";
            when "1100" => return x"43";
            when "1101" => return x"44";
            when "1110" => return x"45";
            when "1111" => return x"46";
            when others => return x"3F";
        end case;
    end function;

    function nibble_mux(cnt : unsigned(1 downto 0); byt : std_logic_vector(7 downto 0))
        return std_logic_vector is
    begin
        case cnt is
            when "00"   => return to_hex(byt(7 downto 4));
            when "01"   => return to_hex(byt(3 downto 0));
            when others => return x"20";
        end case;
    end function;

begin

    numWords_bcd <= reg_numWords;

    -- State register
    state_reg: process(clk)
    begin
        if rising_edge(clk) then
            if reset = '1' then
                state <= IDLE;
            else
                state <= next_state;
            end if;
        end if;
    end process;

    -- Data registers
    data_reg: process(clk)
    begin
        if rising_edge(clk) then
            if reset = '1' then
                echo_reg    <= (others => '0');
                cmd_type    <= CMD_NONE;
                hundreds    <= (others => '0');
                tens        <= (others => '0');
                ones        <= (others => '0');
                digit_count <= (others => '0');
                tx_byte     <= (others => '0');
                nibble_cnt  <= (others => '0');
                byte_idx    <= (others => '0');
                peak_idx    <= (others => '0');
            else
                echo_reg    <= next_echo_reg;
                cmd_type    <= next_cmd_type;
                hundreds    <= next_hundreds;
                tens        <= next_tens;
                ones        <= next_ones;
                digit_count <= next_digit_count;
                tx_byte     <= next_tx_byte;
                nibble_cnt  <= next_nibble_cnt;
                byte_idx    <= next_byte_idx;
                peak_idx    <= next_peak_idx;
            end if;
        end if;
    end process;

    -- numWords_bcd register: latch BCD values when entering START_DP
    numwords_reg: process(clk)
    begin
        if rising_edge(clk) then
            if reset = '1' then
                reg_numWords <= (others => "1001");
            elsif state = ECHO_WAIT and next_state = START_DP then
                reg_numWords(2) <= std_logic_vector(hundreds);
                reg_numWords(1) <= std_logic_vector(tens);
                reg_numWords(0) <= std_logic_vector(ones);
            end if;
        end if;
    end process;

    -- seqDone data capture: latch dataResults and maxIndex on seqDone
    seq_data_capture: process(clk)
    begin
        if rising_edge(clk) then
            if reset = '1' then
                reg_dataResults <= (others => (others => '0'));
                reg_maxIndex    <= (others => (others => '0'));
            elsif seqDone = '1' then
                reg_dataResults <= dataResults;
                reg_maxIndex    <= maxIndex;
            end if;
        end if;
    end process;

    -- seqDone flag: sticky, cleared on START_DP
    seq_flag: process(clk)
    begin
        if rising_edge(clk) then
            if reset = '1' then
                seq_done_flag <= '0';
            elsif state = START_DP then
                seq_done_flag <= '0';
            elsif seqDone = '1' then
                seq_done_flag <= '1';
            end if;
        end if;
    end process;

    -- Combinational logic: next state + outputs
    comb_logic: process(state, rxnow, rxData, txdone, dataReady, byte,
                        seq_done_flag, echo_reg, cmd_type, hundreds, tens, ones,
                        digit_count, tx_byte, nibble_cnt,
                        byte_idx, peak_idx, reg_dataResults, reg_maxIndex)
    begin
        -- Defaults: hold all registers
        next_state       <= state;
        next_echo_reg    <= echo_reg;
        next_cmd_type    <= cmd_type;
        next_hundreds    <= hundreds;
        next_tens        <= tens;
        next_ones        <= ones;
        next_digit_count <= digit_count;
        next_tx_byte     <= tx_byte;
        next_nibble_cnt  <= nibble_cnt;
        next_byte_idx    <= byte_idx;
        next_peak_idx    <= peak_idx;

        -- Output defaults
        rxdone <= '0';
        txnow  <= '0';
        txData <= (others => '0');
        start  <= '0';

        case state is

            -- IDLE: wait for UART character
            when IDLE =>
                if rxnow = '1' then
                    rxdone <= '1';
                    next_echo_reg <= rxData;
                    if rxData = x"41" or rxData = x"61" then
                        -- 'A' or 'a': start command
                        next_cmd_type    <= CMD_A;
                        next_digit_count <= (others => '0');
                    elsif rxData = x"50" or rxData = x"70" then
                        -- 'P' or 'p': peak command
                        next_cmd_type <= CMD_P;
                    elsif rxData = x"4C" or rxData = x"6C" then
                        -- 'L' or 'l': list results command
                        next_cmd_type <= CMD_L;
                    else
                        next_cmd_type <= CMD_NONE;
                    end if;
                    next_state <= ECHO;
                end if;

            -- ECHO: initiate TX of echo character
            when ECHO =>
                txData <= echo_reg;
                txnow  <= '1';
                next_state <= ECHO_WAIT;

            -- ECHO_WAIT: wait for TX done, then branch
            when ECHO_WAIT =>
                txData <= echo_reg;
                if txdone = '1' then
                    case cmd_type is
                        when CMD_NONE =>
                            next_state <= IDLE;
                        when CMD_A =>
                            if digit_count = 3 then
                                next_state <= START_DP;
                            else
                                next_state <= GET_DIGIT;
                            end if;
                        when CMD_P =>
                            next_state <= SEND_LF;
                        when CMD_L =>
                            next_state <= SEND_LF;
                    end case;
                end if;

            -- GET_DIGIT: wait for BCD digit (a command only)
            when GET_DIGIT =>
                if rxnow = '1' then
                    rxdone <= '1';
                    next_echo_reg <= rxData;
                    if rxData >= x"30" and rxData <= x"39" then
                        case digit_count is
                            when "00"   => next_hundreds <= unsigned(rxData(3 downto 0));
                            when "01"   => next_tens     <= unsigned(rxData(3 downto 0));
                            when "10"   => next_ones     <= unsigned(rxData(3 downto 0));
                            when others => null;
                        end case;
                        next_digit_count <= digit_count + 1;
                    else
                        next_cmd_type <= CMD_NONE;
                    end if;
                    next_state <= ECHO;
                end if;

            -- ============================================================
            -- 'a' COMMAND: DATA PROCESSING + STREAMING
            -- ============================================================

            -- START_DP: pulse start, then send newline before streaming
            when START_DP =>
                start <= '1';
                next_nibble_cnt <= (others => '0');
                next_state      <= SEND_LF;

            -- WAIT_DATA: wait for dataReady (byte available from dataConsume)
            when WAIT_DATA =>
                if seq_done_flag = '1' then
                    next_state <= IDLE;
                elsif dataReady = '1' then
                    next_tx_byte    <= byte;
                    next_nibble_cnt <= (others => '0');
                    next_state      <= DATA_TX;
                end if;

            -- DATA_TX: send current nibble/space
            when DATA_TX =>
                txData <= nibble_mux(nibble_cnt, tx_byte);
                txnow  <= '1';
                next_state <= DATA_WAIT;

            -- DATA_WAIT: wait for TX done, advance nibble or next byte
            when DATA_WAIT =>
                txData <= nibble_mux(nibble_cnt, tx_byte);
                if txdone = '1' then
                    if nibble_cnt < 2 then
                        next_nibble_cnt <= nibble_cnt + 1;
                        next_state      <= DATA_TX;
                    else
                        -- All 3 chars sent for this byte
                        next_nibble_cnt <= (others => '0');
                        if seq_done_flag = '1' then
                            next_state <= IDLE;
                        else
                            next_state <= REQ_DATA;
                        end if;
                    end if;
                end if;

            -- REQ_DATA: pulse start to request next byte from dataConsume
            when REQ_DATA =>
                start <= '1';
                next_state <= WAIT_DATA;

            -- ============================================================
            -- NEWLINE (shared by all commands)
            -- ============================================================

            when SEND_LF =>
                txData <= x"0A";
                txnow  <= '1';
                next_state <= SEND_LF_WAIT;

            when SEND_LF_WAIT =>
                txData <= x"0A";
                if txdone = '1' then
                    next_state <= SEND_CR;
                end if;

            when SEND_CR =>
                txData <= x"0D";
                txnow  <= '1';
                next_state <= SEND_CR_WAIT;

            when SEND_CR_WAIT =>
                txData <= x"0D";
                if txdone = '1' then
                    case cmd_type is
                        when CMD_A =>
                            next_state <= WAIT_DATA;
                        when CMD_L =>
                            next_byte_idx   <= (others => '0');
                            next_nibble_cnt <= (others => '0');
                            next_state      <= RES_PREP;
                        when others =>
                            -- CMD_P
                            next_nibble_cnt <= (others => '0');
                            next_state      <= PEAK_PREP;
                    end case;
                end if;

            -- ============================================================
            -- 'l' COMMAND: PRINT 7 RESULT BYTES
            -- ============================================================

            -- RES_PREP: load result byte into tx_byte
            when RES_PREP =>
                next_tx_byte    <= reg_dataResults(to_integer(byte_idx));
                next_nibble_cnt <= (others => '0');
                next_state      <= RES_TX;

            -- RES_TX: send current nibble/space
            when RES_TX =>
                txData <= nibble_mux(nibble_cnt, tx_byte);
                txnow  <= '1';
                next_state <= RES_WAIT;

            -- RES_WAIT: wait for TX done, advance nibble or next byte
            when RES_WAIT =>
                txData <= nibble_mux(nibble_cnt, tx_byte);
                if txdone = '1' then
                    if nibble_cnt < 2 then
                        next_nibble_cnt <= nibble_cnt + 1;
                        next_state      <= RES_TX;
                    else
                        if byte_idx < 6 then
                            next_byte_idx <= byte_idx + 1;
                            next_state    <= RES_PREP;
                        else
                            next_state <= IDLE;
                        end if;
                    end if;
                end if;

            -- ============================================================
            -- 'p' COMMAND: PRINT PEAK BYTE + INDEX
            -- ============================================================

            -- PEAK_PREP: load peak byte (dataResults(3)) into tx_byte
            when PEAK_PREP =>
                next_tx_byte    <= reg_dataResults(3);
                next_nibble_cnt <= (others => '0');
                next_state      <= PEAK_TX;

            -- PEAK_TX: send peak byte nibble/space
            when PEAK_TX =>
                txData <= nibble_mux(nibble_cnt, tx_byte);
                txnow  <= '1';
                next_state <= PEAK_WAIT;

            -- PEAK_WAIT: wait for TX done, then send index digits
            when PEAK_WAIT =>
                txData <= nibble_mux(nibble_cnt, tx_byte);
                if txdone = '1' then
                    if nibble_cnt < 2 then
                        next_nibble_cnt <= nibble_cnt + 1;
                        next_state      <= PEAK_TX;
                    else
                        -- Peak byte done (XX + space), now send 3 index digits
                        next_peak_idx <= (others => '0');
                        next_state    <= PIDX_TX;
                    end if;
                end if;

            -- PIDX_TX: send peak index digit as ASCII
            when PIDX_TX =>
                txData <= x"3" & reg_maxIndex(2 - to_integer(peak_idx));
                txnow  <= '1';
                next_state <= PIDX_WAIT;

            -- PIDX_WAIT: wait for TX done, advance or finish
            when PIDX_WAIT =>
                txData <= x"3" & reg_maxIndex(2 - to_integer(peak_idx));
                if txdone = '1' then
                    if peak_idx < 2 then
                        next_peak_idx <= peak_idx + 1;
                        next_state    <= PIDX_TX;
                    else
                        next_state <= IDLE;
                    end if;
                end if;

            when others =>
                next_state <= IDLE;

        end case;
    end process;

end FSM;
