----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date: 03/07/2023 02:36:21 PM
-- Design Name: 
-- Module Name: test_env - Behavioral
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
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx leaf cells in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity test_env is
    Port ( clk : in STD_LOGIC;
           btn : in STD_LOGIC_VECTOR (4 downto 0);
           sw : in STD_LOGIC_VECTOR (15 downto 0);
           led : out STD_LOGIC_VECTOR (15 downto 0);
           an : out STD_LOGIC_VECTOR (3 downto 0);
           cat : out STD_LOGIC_VECTOR (6 downto 0));
end test_env;

architecture Behavioral of test_env is

component MPG is
    Port ( en : out STD_LOGIC;
           input : in STD_LOGIC;
           clock : in STD_LOGIC);
end component;

component SSD is
    Port ( an : out STD_LOGIC_VECTOR (3 downto 0);
           cat : out STD_LOGIC_VECTOR (6 downto 0);
           digit : in STD_LOGIC_VECTOR (15 downto 0);
           clk : in STD_LOGIC);
end component;

component ID is
      Port(
      clk : in std_logic;
      en : in std_logic;
      Instr : in std_logic_vector(15 downto 0);
      WA: in std_logic_vector(2 downto 0);
      WD : in std_logic_vector(15 downto 0);
      RegWrite : in std_logic;
      
      ExtOp : in std_logic;
      RD1: out std_logic_vector(15 downto 0);
      RD2 : out std_logic_vector(15 downto 0);
      Ext_Imm : out std_logic_vector(15 downto 0);
      func : out std_logic_vector(2 downto 0);
      sa : out std_logic;
      rt : out std_logic_vector(2 downto 0);
      rd : out std_logic_vector(2 downto 0)
      );
end component;

component InstructionFetch is
Port (
Jump : in std_logic;
JumpAddress : in std_logic_vector(15 downto 0);
BranchAddress : in std_logic_vector(15 downto 0);
PCSrc : in std_logic;
En : in std_logic;
Reset : in std_logic;
clk : in std_logic;
Instruction : out std_logic_vector(15 downto 0);
NextAddress : out std_logic_vector (15 downto 0)); --PC+1
end component;

component MainControl is
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
end component;

component EX is
  Port ( 
   RD1: in std_logic_vector(15 downto 0);  
   RD2: in std_logic_vector(15 downto 0);
   ALUSrc: in std_logic;
   Ext_Imm: in std_logic_vector(15 downto 0);
   sa: in std_logic;
   func: in std_logic_vector(2 downto 0);
   ALUOp: in std_logic_vector(2 downto 0);
   PCInc : in std_logic_vector(15 downto 0);
   rt : in std_logic_vector(2 downto 0);
   rd : in std_logic_vector(2 downto 0);
   RegDst : in std_logic;
   Zero: out std_logic;
   ALURes: out std_logic_vector(15 downto 0);
   BranchAddress: out std_logic_vector(15 downto 0);
   rWA: out std_logic_vector(2 downto 0)
  );
end component;

component MEM is
    port ( 
           MemWrite : in STD_LOGIC;	
           ALUResIn : in STD_LOGIC_VECTOR(15 downto 0);
           RD2 : in STD_LOGIC_VECTOR(15 downto 0);		
           clk : in STD_LOGIC;
           en : in STD_LOGIC;
           MemData : out STD_LOGIC_VECTOR(15 downto 0);
           ALUResOut : out STD_LOGIC_VECTOR(15 downto 0));
end component;

--MIPS

signal en : std_logic;
signal en1: std_logic;
signal BranchAddress : std_logic_vector(15 downto 0); 
signal Instruction : std_logic_vector(15 downto 0);
signal NextAddress : std_logic_vector(15 downto 0);
signal digits: std_logic_vector(15 downto 0);
signal RegDst, ExtOp, ALUSrc, Branch, Jump, MemWrite, MemtoReg, RegWrite :  std_logic;
signal ALUOp : std_logic_vector(2 downto 0);
signal RD1, RD2, Ext_Imm : std_logic_vector(15 downto 0);
signal func : std_logic_vector(2 downto 0); 
signal func1 : std_logic_vector(15 downto 0);
signal sa : std_logic;
signal sa1 : std_logic_vector(15 downto 0);
signal WD : std_logic_vector(15 downto 0);
signal ALURes : std_logic_vector(15 downto 0);
signal Zero : std_logic;
signal MemData, ALUResOut : std_logic_vector(15 downto 0);
signal JumpAddress : std_logic_vector(15 downto 0);
signal PCSrc : std_logic;
signal rt, rd, rWA : std_logic_vector(2 downto 0);
signal WA : std_logic_vector(2 downto 0);

--registre pipeline
--IF_ID
signal PC_1_IF_ID, Instruction_IF_ID : std_logic_vector(15 downto 0);
--ID_EX
signal PC_1_ID_EX, Ext_Imm_ID_EX, RD1_ID_EX, RD2_ID_EX : std_logic_vector(15 downto 0);
signal RegDst_ID_EX, ExtOp_ID_EX, ALUSrc_ID_EX, Branch_ID_EX, Jump_ID_EX,
 MemWrite_ID_EX, MemtoReg_ID_EX, RegWrite_ID_EX, sa_ID_EX : std_logic;
signal ALUOp_ID_EX, func_ID_EX, rd_ID_EX, rt_ID_EX : std_logic_vector(2 downto 0);
--EX_MEM
signal Branch_EX_MEM, MemWrite_EX_MEM, MemtoReg_EX_MEM, RegWrite_EX_MEM, Zero_EX_MEM: std_logic;
signal BranchAddress_EX_MEM, ALURes_EX_MEM, RD2_EX_MEM : std_logic_vector(15 downto 0);
signal rd_EX_MEM : std_logic_vector(2 downto 0);
--MEM_WB
signal MemtoReg_MEM_WB, RegWrite_MEM_WB : std_logic;
signal ALURes_MEM_WB, MemData_MEM_WB: std_logic_vector(15 downto 0);
signal rd_MEM_WB : std_logic_vector(2 downto 0);

begin

--MIPS pipeline
--registre

process(clk)
begin
  if rising_edge(clk)then
    if  en = '1' then
      --IF_ID
      PC_1_IF_ID <= NextAddress;
      Instruction_IF_ID <= Instruction;
      --ID_EX
      PC_1_ID_EX <= PC_1_IF_ID;
      RD1_ID_EX <= RD1;
      RD2_ID_EX <= RD2;
      Ext_Imm_ID_EX <= Ext_Imm;
      sa_ID_EX <= sa;
      func_ID_EX <= func;
      rt_ID_EX <= rt;
      rd_ID_EX <= rd;
      MemtoReg_ID_EX <= MemtoReg;
      RegWrite_ID_EX <= RegWrite;
      MemWrite_ID_EX <= MemWrite;
      Branch_ID_EX <= Branch;
      ALUSrc_ID_EX <= ALUSrc;
      ALUOp_ID_EX <= ALUOp;
      RegDst_ID_EX <= RegDst;
      --EX_MEM
      BranchAddress_EX_MEM <= BranchAddress;
      Zero_EX_MEM <= Zero;
      ALURes_EX_MEM <= ALURes;
      RD2_EX_MEM <= RD2_ID_EX;
      rd_EX_MEM <= rWA;
      MemtoReg_EX_MEM <= MemtoReg_ID_EX;
      RegWrite_EX_MEM <= RegWrite_ID_EX;
      Branch_EX_MEM <= Branch_ID_EX;
      --MEM_WB
      MemData_MEM_WB <= MemData;
      ALURes_MEM_WB <= ALURes;
      rd_MEM_WB <= rd_EX_MEM;
      MemtoReg_MEM_WB <= MemtoReg_EX_MEM;
      RegWrite_MEM_WB <= RegWrite_EX_MEM;
   end if;
  end if;
 end process;
 
debouncer : MPG port map(en, btn(0), clk);
debouncer1 : MPG port map(en1, btn(1), clk);


JumpAddress <= PC_1_IF_ID(15 downto 13) & Instruction_IF_ID(12 downto 0);
PCSrc <= Zero_EX_MEM and Branch_EX_MEM;
instrFetch : InstructionFetch port map(Jump, JumpAddress, BranchAddress_EX_MEM, PCSrc, en, en1, clk, Instruction, NextAddress);
main : MainControl port map(Instruction_IF_ID, RegDst, ExtOp, ALUSrc, Branch, Jump, ALUOp, MemWrite, MemtoReg, RegWrite);
decode : ID port map(clk, en, Instruction_IF_ID, rd_MEM_WB, WD, RegWrite_MEM_WB, ExtOp, RD1, RD2, Ext_Imm, func, sa, rt, rd);
execution : EX port map(RD1_ID_EX, RD2_ID_EX, ALUSrc_ID_EX, Ext_Imm_ID_EX, sa_ID_EX, func_ID_EX, ALUOp_ID_EX, PC_1_ID_EX, rt_ID_EX, rd_ID_EX, RegDst_ID_EX, Zero, ALURes, BranchAddress, rWA);
memorie : MEM port map(MemWrite_EX_MEM, ALURes_EX_MEM, RD2_EX_MEM, clk, en, MemData, ALUResOut);

mux_WB : process(MemtoReg_MEM_WB,  ALURes_MEM_WB, MemData_MEM_WB)
         begin
          case MemtoReg_MEM_WB is 
            when '0' => WD <= ALURes_MEM_WB;
            when others => WD <= MemData_MEM_WB;
         end case;
         end process;
         
func1 <= "0000000000000" & func;
sa1 <= "000000000000000" & sa;

led(10 downto 0) <= ALUop & RegDst & ExtOp & ALUSrc & Branch & Jump & MemWrite & MemToReg & RegWrite;

mux : process(sw(7 downto 5), Instruction, NextAddress, RD1, RD2, WD, Ext_Imm, func1, sa1)
      begin
      case sw(7 downto 5) is
        when "000" =>  digits <= Instruction;
        when "001" =>  digits <= NextAddress;
        when "010" =>  digits <= RD1_ID_EX;
        when "011" =>  digits <= RD2_ID_EX;
        when "100" =>  digits <= Ext_Imm_ID_EX;
        when "101" =>  digits <= ALURes;
        when "110" =>  digits <= MemData;
        when "111" =>  digits <= WD;
        when others => digits <= (others => '0');
      end case;
     end process;
     
display : SSD port map(an, cat, digits, clk);

end Behavioral;
