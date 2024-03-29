----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date: 04/05/2023 05:00:19 PM
-- Design Name: 
-- Module Name: UC - Behavioral
-- Project Name: 
-- Target Devices: 
-- Tool Versions: 
-- Description: 
-- 
-- Dependencies: 
-- 
-- Revision:
-- Revision 0.01 - File Created
-- Additional Comments:
-- 
----------------------------------------------------------------------------------


library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx leaf cells in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity MainControl is
   Port ( 
   Instr : in std_logic_vector(15 downto 0);
   RegDst : out std_logic;
   ExtOp : out std_logic;
   ALUSrc : out std_logic;
   Branch : out std_logic;
   Jump : out std_logic;
   ALUOp : out std_logic_vector(2 downto 0);
   MemWrite : out std_logic;
   MemtoReg : out std_logic;
   RegWrite : out std_logic);
end MainControl;

architecture Behavioral of MainControl is

begin

process(Instr(15 downto 13))
begin
RegDst <= '0';
ExtOp <= '0';
ALUSrc <= '0'; 
Branch <= '0';
Jump <= '0'; 
ALUOp <= "000";
MemWrite <= '0';
MemtoReg <= '0'; 
RegWrite <= '0';
case Instr(15 downto 13) is 
      when "000" => --R type
            RegDst <= '1';
            RegWrite <= '1';
            ALUOp <= "000";
      when "001" => --ADDI
            ALUop <= "001";
            ExtOp <= '1';
            ALUSrc <= '1';
            RegWrite <= '1';
      when "010" => --LW
             ALUop <= "001";
             ExtOp <= '1';
             ALUSrc <= '1';
             MemtoReg <= '1';
             RegWrite <= '1';
       when "011" => --SW
             ALUop <= "001";
             ExtOp <= '1';
             ALUSrc <= '1';
             MemWrite <='1';
       when "100" => --BEQ
             ALUop <= "010";
             ExtOp <= '1';
             Branch <= '1';
       when "101" => --ANDI
             ALUop <= "101";
             ExtOp <= '1';
             ALUSrc <= '1';
             RegWrite <= '1';
        when "110" => --ORI
             ALUop <= "110";
             ExtOp <= '1';
             ALUSrc <= '1';
             RegWrite <= '1';
        when "111" => --J
             Jump <= '1';
        when others => 
            RegDst <= '0';
            ExtOp <= '0';
            ALUSrc <= '0'; 
            Branch <= '0';
            Jump <= '0'; 
            ALUOp <= "000";
            MemWrite <= '0';
            MemtoReg <= '0'; 
            RegWrite <= '0';
     end case;
end process;

end Behavioral;
