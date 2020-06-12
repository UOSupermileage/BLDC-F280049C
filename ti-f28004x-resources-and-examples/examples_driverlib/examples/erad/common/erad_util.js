//#############################################################################
//
// erad_util.js
//
// This file contains utility functions to help find function end addresses from
// a map file by calling the function getFuncEndAddress(). All other function 
// definitions in this file are helper functions and should not be used 
// externally. 
//
// Please note that this method of obtaining the end address for a
// function is ONLY supported for applications programmed using C. This method
// will not work for applications programmed using C++ due to function name 
// mangling and other differences that appear in the map file.
//
//#############################################################################

//
// Import DSS and Java packages
//
importPackage(Packages.com.ti.debug.engine.scripting)
importPackage(Packages.com.ti.ccstudio.scripting.environment)
importPackage(Packages.java.lang)
importPackage(Packages.java.util)
importPackage(Packages.java.io)

//
// getFuncStrs - Helper function used to grab the relevant string in the map 
// file that contains the passed string and the next string
//
// Input: Scanner, function name (string)
//
// Output: 2 element array with function address strings
//
function getFuncStrs(sc, funcName)
{
    var nextFuncStr = "";
    var funcStrs    = [];
    var found       = false;
    
    //
    // Loop until we've found the string as long as there are more strings in 
    // the scanner input
    //
    while (sc.hasNextLine() && !found)
    {
        var str = sc.nextLine();
        
        //
        // If the string is found in the input, grab the current and next lines
        // and store into an array
        //
        if(str.contains(funcName))
        {
            nextFuncStr = sc.nextLine();
            funcStrs.push(str);
            funcStrs.push(nextFuncStr);
            found = true;
        }
    }
    
    //
    // Error diagnostic message in case function name was not found in a string 
    // within the map file
    //
    if (!found || funcName == "")
    {
        print("\nERROR: Function name " + funcName + 
              " does not exist in map file\n");
    }
    
    return funcStrs;
}

//
// getFuncAddress - Helper function used to process a string from the map file
// that contains the function name and address. Parses out and returns the 
// relevant address in integer format. The string is expected to be of the
// following form:
// 
// page  address   name                            
// ----  -------   ----                            
// 0     000000f5  ___cinit__ 
//
// Input: function string containing address
//
// Output: function address in integer format
//
function getFuncAddress(funcStr)
{
    //
    // Split the string into words which are separated by spaces
	// s[0] -> page number
	// s[1] -> function address
	// s[2] -> Function name
    //
    var s = funcStr.split(" +")
    
    //
    // Address is a hex value
    //
    return Integer.parseInt(s[1], 16);
}

//
// getSectionInfo - Helper function used to grab the "MEMORY CONFIGURATION" part
// of the map file. This portion of the file ends before the string "SECTION
// ALLOCATION MAP" appears. This portion of the map file  contains information 
// on which sections each address range falls into
//
// Input: Scanner
//
// Output: Array of strings containing the "MEMORY CONFIGURATION" portion of the
// map file
//
function getSectionInfo(sc)
{
    var sectionInfo = [];
    var done        = false;
    
    //
    // Loop through input until "SECTION ALLOCATION MAP" is seen or the input 
    // has run out of strings. Append strings within the "MEMORY CONFIGURATION"
    // section onto the sectionInfo array
    //
    while (sc.hasNextLine() && !done)
    {
        var str = sc.nextLine();
        
        if (str.contains("SECTION ALLOCATION MAP"))
        {
            done = true;
        }
        else
        {
            sectionInfo.push(str);
        }
    }
    
    return sectionInfo;
}

//
// getFuncPageNo - Helper function used to grab the page value that a specific
// function's address falls into. If the input is a valid string, then the first
// character of the passed string is always the page number
//
// Input: Function string
//
// Output: Page number (integer)
//
function getFuncPageNo(funcStr)
{
    //
    // Default -1 for page numbers, unless a valid page value is found
    //
    var pageNo = -1;

    //
    // If the string is not empty and is not a special case containing "abs",
    // grab the first character and return the integer as the page number
    //
    if(funcStr.length() > 0 && !funcStr.startsWith("abs"))
    {
        pageNo = Integer.parseInt(funcStr.charAt(0));
    }
    
    return pageNo;
}

//
// getLastAddrSection - Helper funciton used to grab the final address within
// a specific section as specificed in the "MEMORY CONFIGURATION" section of a
// map file. The final address is calculated as the following sum:
// last address = origin address + length of section - unused portion
//
// Input: Function string, section info strings from "MEMORY CONFIGURATION" 
// section
//       
// Output: The last address that appears within the relevant section
//
function getLastAddrSection(funcStr, sectionInfo)
{
    //
    // Expect that the string will break up into six sections if the input is
    // formatted as expected
    //
    var lastAddr = 0x0;
    var found    = false;
    var expectedSectionNum = 6;
    
    //
    // Grab the start address for this function
    //
    var funcAddress = getFuncAddress(funcStr);
    var i = 0;

    //  
    // Loop over "origin" addresses and find correct section that the function
    // falls into. Stop looping if the last address has been calculated or the
    // input has run out of strings
    //
    while (!found && i < sectionInfo.length)
    {
        //
        // Current string to be processed
        //
        var curStr = sectionInfo[i];
        
        //
        // Perform a regular expression match to break string up based on a
        // number of separating spaces. Match will return null if empty string
        // is used and an empty array is returned in this case
        //
        var sections = curStr.match(/\S+/g) || [];
        
        var originAddress = 0x0;
        var sectionLength = 0x0;
        var unused        = 0x0;
        if(sections.length == expectedSectionNum)
        { 
            // 
            // Grab the relevant sections. 2nd portion is the origin, 3rd 
            // portion is the length and the 5th portion is the unsued amount
            //
            originAddress = Integer.parseInt(sections[1], 16);
            sectionLength = Integer.parseInt(sections[2], 16);
            unused        = Integer.parseInt(sections[4], 16);
        }
        
        //
        // If the function's address fits in this range, then the correct 
        // section is found. Calculate the final address in this section
        //
        if(funcAddress < originAddress + sectionLength &&
           funcAddress > originAddress)
        {
            lastAddr = originAddress + sectionLength - unused;
            found = true;       
        }
        
        //
        // Increment indicating that a string from sectionInfo has been
        // processed
        //
        i++;
    }
    
    return lastAddr;
}

//
// getFuncAddress - Top level function used to grab the end address for a 
// specific function. In this file, this is the only function that should be
// used externally. All other funciton definitions are helper functions used 
// internally
//
// Input: Function name (string)
//
// Output: Function end address (integer)
//
function getFuncEndAddress(funcName)
{
    var endAddress = 0x0;
    
    //
    // Check for the existence of script environment variables. An error message
    // is printed if the string has not been defined in the current scripting
    // session. Note: These error messages will not trigger if the variables
    // are defined, but are set to incorrect values. It is up to the user to
    // make sure that each of these variables is set accordingly prior to
    // running the example program
    //
    if (typeof PROJ_NAME == 'undefined')
    {
        print("\nERROR: Please set PROJ_NAME to the name of the" + 
              " CCS project\n");
    }
    
    if (typeof PROJ_WKSPC_LOC == 'undefined')
    {
        print("\nERROR: Please set PROJ_WKSPC_LOC to the correct" + 
              " CCS workspace path\n");
    }
    
    if (typeof PROJ_CONFIG == 'undefined')
    {
        print("\nERROR: Please set PROJ_CONFIG to the name of the" + 
              " active configuration\n");
    }
    
    //
    // Open map file whose location is based on the pre-set script environment
    // variables
    //
    var mapFile = 
        new File(PROJ_WKSPC_LOC + "\\" + PROJ_NAME + "\\" + PROJ_CONFIG +
                 "\\" + PROJ_NAME + ".map"); 
    var sc = new Scanner(mapFile);
    var sectionInfo      = [];
    var funcAddrFound    = false;
    var sectionInfoFound = false;

    //
    // Loop through the map file until the end function address is found or the
    // input has run out of strings
    //
    while (sc.hasNextLine() && !funcAddrFound)
    {
        var str = sc.nextLine();
        
        //
        // Section info found within the portion labled "MEMORY CONFIGURATION"
        // starts with "PAGE 0:". Save off this portion of the input
        //
        if(str.contains("PAGE 0:"))
        {
            sectionInfo = getSectionInfo(sc);
        }
                
        //
        // Continue parsing the map file until the sorted list of symbol 
        // addresses is found
        //
        if(str.contains("GLOBAL SYMBOLS: SORTED BY Symbol Address"))
        {
            //
            // Grab the string containing this function's name and the next 
            // string
            //
            var funcStrs          = getFuncStrs(sc, funcName);
            
            //
            // Grab and compare both this function's page number and the next
            // function's page number. If they differ, the next function's 
            // address cannot be used to calculate the end address of the first
            // function
            //
            var funcStrPageNo     = getFuncPageNo(funcStrs[0]);
            var nextFuncStrPageNo = getFuncPageNo(funcStrs[1]);
            
            //
            // Grab function end address based on page number for both this
            // function and the next. Subtract 0x2 from the address to ensure
            // that the address is of the function's final instruction
            //
            if(funcStrPageNo == -1)
            {
                print ("\nERROR: Function does not exist in a valid Page" + 
                       " (0 or 1)\n");
            }
            else if(funcStrPageNo == nextFuncStrPageNo)
            {
                //
                // If both this function and the next function exist in the 
                // same page, then the next function's address can be used to 
                // calculate the end address of this function
                //
                var nextFuncAddress = getFuncAddress(funcStrs[1]);
                endAddress          = nextFuncAddress - 0x2;
            }
            else
            {
                // 
                // If this function exists within a different page than the next
                // function, then the final address in the specific memory 
                // section must be used to determine the end address
                //
                endAddress = getLastAddrSection(funcStrs[0], sectionInfo) - 0x2;
            }
            
            funcAddrFound = true;
        }
    }
 
    //
    // Return end address as an integer
    //
    return endAddress;
}

//
// End of File
//