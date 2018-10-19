  function targMap = targDataMap(),

  ;%***********************
  ;% Create Parameter Map *
  ;%***********************
      
    nTotData      = 0; %add to this count as we go
    nTotSects     = 2;
    sectIdxOffset = 0;
    
    ;%
    ;% Define dummy sections & preallocate arrays
    ;%
    dumSection.nData = -1;  
    dumSection.data  = [];
    
    dumData.logicalSrcIdx = -1;
    dumData.dtTransOffset = -1;
    
    ;%
    ;% Init/prealloc paramMap
    ;%
    paramMap.nSections           = nTotSects;
    paramMap.sectIdxOffset       = sectIdxOffset;
      paramMap.sections(nTotSects) = dumSection; %prealloc
    paramMap.nTotData            = -1;
    
    ;%
    ;% Auto data (rtP)
    ;%
      section.nData     = 81;
      section.data(81)  = dumData; %prealloc
      
	  ;% rtP.QueryInstrument_P1_Size
	  section.data(1).logicalSrcIdx = 0;
	  section.data(1).dtTransOffset = 0;
	
	  ;% rtP.QueryInstrument_P1
	  section.data(2).logicalSrcIdx = 1;
	  section.data(2).dtTransOffset = 2;
	
	  ;% rtP.QueryInstrument_P2_Size
	  section.data(3).logicalSrcIdx = 2;
	  section.data(3).dtTransOffset = 3;
	
	  ;% rtP.QueryInstrument_P2
	  section.data(4).logicalSrcIdx = 3;
	  section.data(4).dtTransOffset = 5;
	
	  ;% rtP.QueryInstrument_P3_Size
	  section.data(5).logicalSrcIdx = 4;
	  section.data(5).dtTransOffset = 6;
	
	  ;% rtP.QueryInstrument_P3
	  section.data(6).logicalSrcIdx = 5;
	  section.data(6).dtTransOffset = 8;
	
	  ;% rtP.QueryInstrument_P4_Size
	  section.data(7).logicalSrcIdx = 6;
	  section.data(7).dtTransOffset = 9;
	
	  ;% rtP.QueryInstrument_P4
	  section.data(8).logicalSrcIdx = 7;
	  section.data(8).dtTransOffset = 11;
	
	  ;% rtP.QueryInstrument_P5_Size
	  section.data(9).logicalSrcIdx = 8;
	  section.data(9).dtTransOffset = 12;
	
	  ;% rtP.QueryInstrument_P5
	  section.data(10).logicalSrcIdx = 9;
	  section.data(10).dtTransOffset = 14;
	
	  ;% rtP.QueryInstrument_P6_Size
	  section.data(11).logicalSrcIdx = 10;
	  section.data(11).dtTransOffset = 15;
	
	  ;% rtP.QueryInstrument_P6
	  section.data(12).logicalSrcIdx = 11;
	  section.data(12).dtTransOffset = 17;
	
	  ;% rtP.QueryInstrument_P7_Size
	  section.data(13).logicalSrcIdx = 12;
	  section.data(13).dtTransOffset = 18;
	
	  ;% rtP.QueryInstrument_P7
	  section.data(14).logicalSrcIdx = 13;
	  section.data(14).dtTransOffset = 20;
	
	  ;% rtP.QueryInstrument_P8_Size
	  section.data(15).logicalSrcIdx = 14;
	  section.data(15).dtTransOffset = 21;
	
	  ;% rtP.QueryInstrument_P8
	  section.data(16).logicalSrcIdx = 15;
	  section.data(16).dtTransOffset = 23;
	
	  ;% rtP.QueryInstrument_P9_Size
	  section.data(17).logicalSrcIdx = 16;
	  section.data(17).dtTransOffset = 24;
	
	  ;% rtP.QueryInstrument_P9
	  section.data(18).logicalSrcIdx = 17;
	  section.data(18).dtTransOffset = 26;
	
	  ;% rtP.QueryInstrument_P10_Size
	  section.data(19).logicalSrcIdx = 18;
	  section.data(19).dtTransOffset = 27;
	
	  ;% rtP.QueryInstrument_P10
	  section.data(20).logicalSrcIdx = 19;
	  section.data(20).dtTransOffset = 29;
	
	  ;% rtP.QueryInstrument_P11_Size
	  section.data(21).logicalSrcIdx = 20;
	  section.data(21).dtTransOffset = 30;
	
	  ;% rtP.QueryInstrument_P11
	  section.data(22).logicalSrcIdx = 21;
	  section.data(22).dtTransOffset = 32;
	
	  ;% rtP.QueryInstrument_P12_Size
	  section.data(23).logicalSrcIdx = 22;
	  section.data(23).dtTransOffset = 33;
	
	  ;% rtP.QueryInstrument_P12
	  section.data(24).logicalSrcIdx = 23;
	  section.data(24).dtTransOffset = 35;
	
	  ;% rtP.QueryInstrument_P13_Size
	  section.data(25).logicalSrcIdx = 24;
	  section.data(25).dtTransOffset = 36;
	
	  ;% rtP.QueryInstrument_P13
	  section.data(26).logicalSrcIdx = 25;
	  section.data(26).dtTransOffset = 38;
	
	  ;% rtP.QueryInstrument_P14_Size
	  section.data(27).logicalSrcIdx = 26;
	  section.data(27).dtTransOffset = 39;
	
	  ;% rtP.QueryInstrument_P14
	  section.data(28).logicalSrcIdx = 27;
	  section.data(28).dtTransOffset = 41;
	
	  ;% rtP.QueryInstrument_P15_Size
	  section.data(29).logicalSrcIdx = 28;
	  section.data(29).dtTransOffset = 42;
	
	  ;% rtP.QueryInstrument_P15
	  section.data(30).logicalSrcIdx = 29;
	  section.data(30).dtTransOffset = 44;
	
	  ;% rtP.QueryInstrument_P16_Size
	  section.data(31).logicalSrcIdx = 30;
	  section.data(31).dtTransOffset = 45;
	
	  ;% rtP.QueryInstrument_P16
	  section.data(32).logicalSrcIdx = 31;
	  section.data(32).dtTransOffset = 47;
	
	  ;% rtP.QueryInstrument_P17_Size
	  section.data(33).logicalSrcIdx = 32;
	  section.data(33).dtTransOffset = 56;
	
	  ;% rtP.QueryInstrument_P17
	  section.data(34).logicalSrcIdx = 33;
	  section.data(34).dtTransOffset = 58;
	
	  ;% rtP.QueryInstrument_P18_Size
	  section.data(35).logicalSrcIdx = 34;
	  section.data(35).dtTransOffset = 59;
	
	  ;% rtP.QueryInstrument_P18
	  section.data(36).logicalSrcIdx = 35;
	  section.data(36).dtTransOffset = 61;
	
	  ;% rtP.QueryInstrument_P19_Size
	  section.data(37).logicalSrcIdx = 36;
	  section.data(37).dtTransOffset = 70;
	
	  ;% rtP.QueryInstrument_P19
	  section.data(38).logicalSrcIdx = 37;
	  section.data(38).dtTransOffset = 72;
	
	  ;% rtP.QueryInstrument_P20_Size
	  section.data(39).logicalSrcIdx = 38;
	  section.data(39).dtTransOffset = 73;
	
	  ;% rtP.QueryInstrument_P20
	  section.data(40).logicalSrcIdx = 39;
	  section.data(40).dtTransOffset = 75;
	
	  ;% rtP.QueryInstrument_P21_Size
	  section.data(41).logicalSrcIdx = 40;
	  section.data(41).dtTransOffset = 85;
	
	  ;% rtP.QueryInstrument_P21
	  section.data(42).logicalSrcIdx = 41;
	  section.data(42).dtTransOffset = 87;
	
	  ;% rtP.QueryInstrument_P22_Size
	  section.data(43).logicalSrcIdx = 42;
	  section.data(43).dtTransOffset = 88;
	
	  ;% rtP.QueryInstrument_P22
	  section.data(44).logicalSrcIdx = 43;
	  section.data(44).dtTransOffset = 90;
	
	  ;% rtP.QueryInstrument_P23_Size
	  section.data(45).logicalSrcIdx = 44;
	  section.data(45).dtTransOffset = 105;
	
	  ;% rtP.QueryInstrument_P23
	  section.data(46).logicalSrcIdx = 45;
	  section.data(46).dtTransOffset = 107;
	
	  ;% rtP.QueryInstrument_P24_Size
	  section.data(47).logicalSrcIdx = 46;
	  section.data(47).dtTransOffset = 110;
	
	  ;% rtP.QueryInstrument_P25_Size
	  section.data(48).logicalSrcIdx = 48;
	  section.data(48).dtTransOffset = 112;
	
	  ;% rtP.QueryInstrument_P25
	  section.data(49).logicalSrcIdx = 49;
	  section.data(49).dtTransOffset = 114;
	
	  ;% rtP.QueryInstrument_P26_Size
	  section.data(50).logicalSrcIdx = 50;
	  section.data(50).dtTransOffset = 115;
	
	  ;% rtP.QueryInstrument_P26
	  section.data(51).logicalSrcIdx = 51;
	  section.data(51).dtTransOffset = 117;
	
	  ;% rtP.QueryInstrument_P27_Size
	  section.data(52).logicalSrcIdx = 52;
	  section.data(52).dtTransOffset = 127;
	
	  ;% rtP.QueryInstrument_P27
	  section.data(53).logicalSrcIdx = 53;
	  section.data(53).dtTransOffset = 129;
	
	  ;% rtP.QueryInstrument_P28_Size
	  section.data(54).logicalSrcIdx = 54;
	  section.data(54).dtTransOffset = 130;
	
	  ;% rtP.QueryInstrument_P28
	  section.data(55).logicalSrcIdx = 55;
	  section.data(55).dtTransOffset = 132;
	
	  ;% rtP.QueryInstrument_P29_Size
	  section.data(56).logicalSrcIdx = 56;
	  section.data(56).dtTransOffset = 133;
	
	  ;% rtP.QueryInstrument_P29
	  section.data(57).logicalSrcIdx = 57;
	  section.data(57).dtTransOffset = 135;
	
	  ;% rtP.QueryInstrument_P30_Size
	  section.data(58).logicalSrcIdx = 58;
	  section.data(58).dtTransOffset = 136;
	
	  ;% rtP.QueryInstrument_P30
	  section.data(59).logicalSrcIdx = 59;
	  section.data(59).dtTransOffset = 138;
	
	  ;% rtP.QueryInstrument_P31_Size
	  section.data(60).logicalSrcIdx = 60;
	  section.data(60).dtTransOffset = 139;
	
	  ;% rtP.QueryInstrument_P31
	  section.data(61).logicalSrcIdx = 61;
	  section.data(61).dtTransOffset = 141;
	
	  ;% rtP.QueryInstrument_P32_Size
	  section.data(62).logicalSrcIdx = 62;
	  section.data(62).dtTransOffset = 142;
	
	  ;% rtP.QueryInstrument_P32
	  section.data(63).logicalSrcIdx = 63;
	  section.data(63).dtTransOffset = 144;
	
	  ;% rtP.QueryInstrument_P33_Size
	  section.data(64).logicalSrcIdx = 64;
	  section.data(64).dtTransOffset = 145;
	
	  ;% rtP.QueryInstrument_P33
	  section.data(65).logicalSrcIdx = 65;
	  section.data(65).dtTransOffset = 147;
	
	  ;% rtP.QueryInstrument_P34_Size
	  section.data(66).logicalSrcIdx = 66;
	  section.data(66).dtTransOffset = 148;
	
	  ;% rtP.QueryInstrument_P34
	  section.data(67).logicalSrcIdx = 67;
	  section.data(67).dtTransOffset = 150;
	
	  ;% rtP.QueryInstrument_P35_Size
	  section.data(68).logicalSrcIdx = 68;
	  section.data(68).dtTransOffset = 151;
	
	  ;% rtP.QueryInstrument_P35
	  section.data(69).logicalSrcIdx = 69;
	  section.data(69).dtTransOffset = 153;
	
	  ;% rtP.QueryInstrument_P36_Size
	  section.data(70).logicalSrcIdx = 70;
	  section.data(70).dtTransOffset = 162;
	
	  ;% rtP.QueryInstrument_P36
	  section.data(71).logicalSrcIdx = 71;
	  section.data(71).dtTransOffset = 164;
	
	  ;% rtP.QueryInstrument_P37_Size
	  section.data(72).logicalSrcIdx = 72;
	  section.data(72).dtTransOffset = 165;
	
	  ;% rtP.QueryInstrument_P37
	  section.data(73).logicalSrcIdx = 73;
	  section.data(73).dtTransOffset = 167;
	
	  ;% rtP.QueryInstrument_P38_Size
	  section.data(74).logicalSrcIdx = 74;
	  section.data(74).dtTransOffset = 168;
	
	  ;% rtP.QueryInstrument_P38
	  section.data(75).logicalSrcIdx = 75;
	  section.data(75).dtTransOffset = 170;
	
	  ;% rtP.QueryInstrument_P39_Size
	  section.data(76).logicalSrcIdx = 76;
	  section.data(76).dtTransOffset = 171;
	
	  ;% rtP.QueryInstrument_P39
	  section.data(77).logicalSrcIdx = 77;
	  section.data(77).dtTransOffset = 173;
	
	  ;% rtP.SimulationPace_P1
	  section.data(78).logicalSrcIdx = 78;
	  section.data(78).dtTransOffset = 174;
	
	  ;% rtP.SimulationPace_P2
	  section.data(79).logicalSrcIdx = 79;
	  section.data(79).dtTransOffset = 175;
	
	  ;% rtP.SimulationPace_P3
	  section.data(80).logicalSrcIdx = 80;
	  section.data(80).dtTransOffset = 176;
	
	  ;% rtP.SimulationPace_P4
	  section.data(81).logicalSrcIdx = 81;
	  section.data(81).dtTransOffset = 177;
	
      nTotData = nTotData + section.nData;
      paramMap.sections(1) = section;
      clear section
      
      section.nData     = 1;
      section.data(1)  = dumData; %prealloc
      
	  ;% rtP.Gain_Gain
	  section.data(1).logicalSrcIdx = 82;
	  section.data(1).dtTransOffset = 0;
	
      nTotData = nTotData + section.nData;
      paramMap.sections(2) = section;
      clear section
      
    
      ;%
      ;% Non-auto Data (parameter)
      ;%
    

    ;%
    ;% Add final counts to struct.
    ;%
    paramMap.nTotData = nTotData;
    


  ;%**************************
  ;% Create Block Output Map *
  ;%**************************
      
    nTotData      = 0; %add to this count as we go
    nTotSects     = 2;
    sectIdxOffset = 0;
    
    ;%
    ;% Define dummy sections & preallocate arrays
    ;%
    dumSection.nData = -1;  
    dumSection.data  = [];
    
    dumData.logicalSrcIdx = -1;
    dumData.dtTransOffset = -1;
    
    ;%
    ;% Init/prealloc sigMap
    ;%
    sigMap.nSections           = nTotSects;
    sigMap.sectIdxOffset       = sectIdxOffset;
      sigMap.sections(nTotSects) = dumSection; %prealloc
    sigMap.nTotData            = -1;
    
    ;%
    ;% Auto data (rtB)
    ;%
      section.nData     = 1;
      section.data(1)  = dumData; %prealloc
      
	  ;% rtB.e51hxnpgyt
	  section.data(1).logicalSrcIdx = 0;
	  section.data(1).dtTransOffset = 0;
	
      nTotData = nTotData + section.nData;
      sigMap.sections(1) = section;
      clear section
      
      section.nData     = 1;
      section.data(1)  = dumData; %prealloc
      
	  ;% rtB.e4tzxunoxe
	  section.data(1).logicalSrcIdx = 1;
	  section.data(1).dtTransOffset = 0;
	
      nTotData = nTotData + section.nData;
      sigMap.sections(2) = section;
      clear section
      
    
      ;%
      ;% Non-auto Data (signal)
      ;%
    

    ;%
    ;% Add final counts to struct.
    ;%
    sigMap.nTotData = nTotData;
    


  ;%*******************
  ;% Create DWork Map *
  ;%*******************
      
    nTotData      = 0; %add to this count as we go
    nTotSects     = 1;
    sectIdxOffset = 2;
    
    ;%
    ;% Define dummy sections & preallocate arrays
    ;%
    dumSection.nData = -1;  
    dumSection.data  = [];
    
    dumData.logicalSrcIdx = -1;
    dumData.dtTransOffset = -1;
    
    ;%
    ;% Init/prealloc dworkMap
    ;%
    dworkMap.nSections           = nTotSects;
    dworkMap.sectIdxOffset       = sectIdxOffset;
      dworkMap.sections(nTotSects) = dumSection; %prealloc
    dworkMap.nTotData            = -1;
    
    ;%
    ;% Auto data (rtDW)
    ;%
      section.nData     = 3;
      section.data(3)  = dumData; %prealloc
      
	  ;% rtDW.pth3iimf1a
	  section.data(1).logicalSrcIdx = 0;
	  section.data(1).dtTransOffset = 0;
	
	  ;% rtDW.pb5lgtcc5y.LoggedData
	  section.data(2).logicalSrcIdx = 1;
	  section.data(2).dtTransOffset = 1;
	
	  ;% rtDW.cmfu1hogpu.LoggedData
	  section.data(3).logicalSrcIdx = 2;
	  section.data(3).dtTransOffset = 3;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(1) = section;
      clear section
      
    
      ;%
      ;% Non-auto Data (dwork)
      ;%
    

    ;%
    ;% Add final counts to struct.
    ;%
    dworkMap.nTotData = nTotData;
    


  ;%
  ;% Add individual maps to base struct.
  ;%

  targMap.paramMap  = paramMap;    
  targMap.signalMap = sigMap;
  targMap.dworkMap  = dworkMap;
  
  ;%
  ;% Add checksums to base struct.
  ;%


  targMap.checksum0 = 1117595932;
  targMap.checksum1 = 2452019217;
  targMap.checksum2 = 2211008495;
  targMap.checksum3 = 3397671295;

