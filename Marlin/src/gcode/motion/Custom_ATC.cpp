#include "../gcode.h"
#include "../../module/motion.h"
#include "../../lcd/marlinui.h"
#include "../../inc/MarlinConfig.h"

// Tyto proměnné nám dají přístup k aktuální fyzické poloze ramene
extern xyze_pos_t current_position; 

ToolData tool_table[16];

// ==========================================
// 1. HARDWAROVÉ NASTAVENÍ (PINY A LOGIKA)
// ==========================================
#define TOOL_ID_BIT0_PIN   PE12  
#define TOOL_ID_BIT1_PIN   PE13  
#define TOOL_ID_BIT2_PIN   PE14  
#define TOOL_ID_BIT3_PIN   PE15  

#define DOCK_0_SENSOR_PIN PG11
#define DOCK_1_SENSOR_PIN PG12
#define DOCK_2_SENSOR_PIN PG13

#define HEAD_EMPTY 15 
#define HEAD_ERROR 0  

// ==========================================
// 2. PAMĚŤ, DATABÁZE A TEACH-IN
// ==========================================
struct ToolDock { float a_offset; };
const ToolDock magazine[3] = { 
    { -22.0 }, // Dock 0 offset
    {   0.0 }, // Dock 1 offset (Referenční střed)
    {  22.0 }  // Dock 2 offset
};

int8_t tool_in_hand = -1;                 
int8_t dock_tool[3] = { -1, -1, -1 };     

// --- PROMĚNNÉ PRO KALIBRACI (TEACH-IN) ---
bool is_atc_calibrated = false;
float ref_R = 0.0;
float ref_A = 0.0;
float ref_Z = 0.0;

// ==========================================
// 3. POMOCNÉ FUNKCE
// ==========================================
uint8_t read_pogo_pins() {
    uint8_t b0 = READ(TOOL_ID_BIT0_PIN);
    uint8_t b1 = READ(TOOL_ID_BIT1_PIN);
    uint8_t b2 = READ(TOOL_ID_BIT2_PIN);
    uint8_t b3 = READ(TOOL_ID_BIT3_PIN);
    return (b3 << 3) | (b2 << 2) | (b1 << 1) | b0;
}

void print_binary_4bit(uint8_t val) {
    for (int i = 3; i >= 0; i--) SERIAL_ECHO((val & (1 << i)) ? 1 : 0);
}

int read_dock_sensor(uint8_t d) {
  if (d == 0) return READ(DOCK_0_SENSOR_PIN);
  if (d == 1) return READ(DOCK_1_SENSOR_PIN);
  if (d == 2) return READ(DOCK_2_SENSOR_PIN);
  return 1; 
}

void exec_move(const char* cmd_prefix, float a, float r, float z, int f = 0) {
    char cmd[128];
    char sA[15], sR[15], sZ[15];
    
    dtostrf(a, 4, 2, sA);
    dtostrf(r, 4, 2, sR);
    dtostrf(z, 4, 2, sZ);
    
    if (f > 0) sprintf(cmd, "%s A%s R%s Z%s F%d", cmd_prefix, sA, sR, sZ, f);
    else sprintf(cmd, "%s A%s R%s Z%s", cmd_prefix, sA, sR, sZ);
    
    gcode.process_subcommands_now(cmd);
}

// --- OPTIMALIZACE: UNIVERZÁLNÍ FUNKCE PRO ODLOŽENÍ ---
bool unload_current_tool() {

    // Zjistíme, zda jsou souřadnice v relativním módu (stačí test na X osu)
    bool was_relative = GcodeSuite::axis_is_relative(X_AXIS);
    
    // Pokud ano, přepneme vše na absolutní mód (G90)
    if (was_relative) {
        GcodeSuite::set_relative_mode(false);
    }


    if (tool_in_hand == -1) return true; // Ruka už je prázdná

    int8_t free_dock = -1;
    float min_dist = 9999.0;
    float cur_A = DEGREES(atan2(current_position.y, current_position.x));

    // Najdeme fyzicky i logicky volný dock, který je nejblíž
    for (int i = 0; i < 3; i++) { 
        if (dock_tool[i] == -1 && read_dock_sensor(i) == 0) { 
            float dock_A = ref_A + magazine[i].a_offset;
            float dist = fabs(cur_A - dock_A);
            if (dist > 180.0) dist = 360.0 - dist;

            if (dist < min_dist) {
                min_dist = dist;
                free_dock = i;
            }
        } 
    }

    if (free_dock == -1) { 
        SERIAL_ECHOLNPGM("FATAL: Neni fyzicky volny dock pro odlozeni nastroje!"); 
        return false; 
    }

    float t_A = ref_A + magazine[free_dock].a_offset;

    SERIAL_ECHOPGM(">> Odkladam T"); SERIAL_ECHO(tool_in_hand); SERIAL_ECHOPGM(" do D"); SERIAL_ECHOLN(free_dock);
    
    float current_A = atan2(current_position.y, current_position.x) * (180.0 / M_PI);
    float current_R = hypot(current_position.x, current_position.y); // Odpovídá sqrt(X^2 + Y^2)
    
    exec_move("G8", current_A, current_R, ref_Z + 100);
    exec_move("G8", current_A, ref_R + 100, ref_Z + 100);
    exec_move("G8", t_A, ref_R + 100, ref_Z + 100);       
    exec_move("G8", t_A, ref_R + 60,  ref_Z + 6, 2000);   
    exec_move("G9", t_A, ref_R,       ref_Z, 1000);       
    
    safe_delay(1000); gcode.process_subcommands_now((char *)"M280 P0 S180"); safe_delay(1000); 
    
    exec_move("G9", t_A, ref_R,       ref_Z + 60, 1500);     

    dock_tool[free_dock] = tool_in_hand; 
    tool_in_hand = -1;

    // Obnova po úspěšném dokončení - vrátíme zpět na relativní, pokud byl
    if (was_relative) GcodeSuite::set_relative_mode(true);

    return true;
}

// ==========================================
// 4. M666: SPRÁVA KNIHOVNY NÁSTROJŮ
// ==========================================
void GcodeSuite::M666() {
    int id = -1;

    // OPRAVA 1: 'seenval' ignoruje písmena uvnitř textu a reaguje jen na P následované číslem
    if (parser.seenval('P')) {
        id = parser.value_int();
    } 
    else if (parser.seen('S') || parser.seenval('X') || parser.seenval('Y') || parser.seenval('Z') || parser.seenval('C')) {
        uint8_t head_id = read_pogo_pins();
        if (head_id != HEAD_EMPTY && head_id != HEAD_ERROR) {
            id = head_id; 
        } else {
            SERIAL_ECHOLNPGM("CHYBA: Neni zadano P<id> a v ruce neni nastroj k uprave!");
            return;
        }
    }

    if (id >= 0 && id < 16 && (parser.seen('S') || parser.seenval('X') || parser.seenval('Y') || parser.seenval('Z') || parser.seenval('C'))) {
        
        // Všude použito seenval() pro bezpečné vytažení číselných hodnot
        if (parser.seenval('X')) tool_table[id].lx = parser.value_float();
        if (parser.seenval('Y')) tool_table[id].ly = parser.value_float();
        if (parser.seenval('Z')) tool_table[id].lz = parser.value_float();
        if (parser.seenval('C')) tool_table[id].clamp_angle = parser.value_int(); 
        
        // OPRAVA 2: Čisté vyčtení jména pomocí veřejné proměnné parser.string_arg
        if (parser.seen('S')) {
            char* str = parser.string_arg; 
            if (str) {
                int j = 0;
                bool in_quotes = false;
                
                // 1. Přeskočíme všechny počáteční mezery a případné samotné písmeno 'S'
                int i = 0;
                while (str[i] == ' ' || str[i] == 'S') {
                    i++;
                }
                
                // 2. Samotné čtení slova
                for (; str[i] != '\0' && j < 14; i++) {
                    if (str[i] == '"') {
                        in_quotes = !in_quotes; // Přepínáme stav uvnitř/vně uvozovek
                        continue; // Samotné uvozovky nekopírujeme
                    }
                    
                    // Konec slova (narazili jsme na mezeru za slovem a nejsme v uvozovkách)
                    if (str[i] == ' ' && !in_quotes) break; 
                    
                    tool_table[id].name[j++] = str[i];
                }
                tool_table[id].name[j] = '\0'; // Uzavření řetězce
            }
        }
        
        SERIAL_ECHOPGM("Nastroj T"); SERIAL_ECHO(id); 
        SERIAL_ECHOLNPGM(" ulozen v RAM. Pro EEPROM zadejte M500.");
    } 
    else if (!parser.seen('S') && !parser.seenval('X') && !parser.seenval('Y') && !parser.seenval('Z') && !parser.seenval('C')) {
        
        // Čistý výpis knihovny (bez uvozovek a 'S' v paměti)
        SERIAL_ECHOLNPGM("Knihovna nastroju:");
        for (uint8_t i = 0; i < 16; i++) {
            if (tool_table[i].name[0] != '\0') {
                SERIAL_ECHOPGM("T"); SERIAL_ECHO(i); SERIAL_ECHOPGM(" ("); SERIAL_ECHO(tool_table[i].name);
                SERIAL_ECHOPGM(") -> X:"); SERIAL_ECHO(tool_table[i].lx); 
                SERIAL_ECHOPGM(" Y:"); SERIAL_ECHO(tool_table[i].ly); 
                SERIAL_ECHOPGM(" Z:"); SERIAL_ECHOLN(tool_table[i].lz);
                SERIAL_ECHOPGM(" Uhel:"); SERIAL_ECHOLN(tool_table[i].clamp_angle); 
            }
        }
    }
}

// ==========================================
// 5. M667: MASTER SPRÁVCE ATC & AUTO-SCAN
// ==========================================
void GcodeSuite::M667() {
    
    // --- C) TEACH-IN KALIBRACE ---
    if (parser.seen('C')) {
        int cal_dock = parser.value_int();
        if (cal_dock >= 0 && cal_dock < 3) {
            float cur_x = current_position.x;
            float cur_y = current_position.y;
            float cur_A = DEGREES(atan2(cur_y, cur_x));
            
            ref_A = cur_A - magazine[cal_dock].a_offset;
            // OPTIMALIZACE: Rychlejší výpočet přepony
            ref_R = HYPOT(cur_x, cur_y); 
            ref_Z = current_position.z;
            is_atc_calibrated = true;
            
            SERIAL_ECHOLNPGM("=======================================");
            SERIAL_ECHOLNPGM(">>> ATC USPESNE NAKALIBROVANO <<<");
            SERIAL_ECHOPGM("Kalibrovano pomoci Hnizda: "); SERIAL_ECHOLN(cal_dock);
            SERIAL_ECHOPGM("Vypocteny stred (Dock 1) -> R:"); SERIAL_ECHO(ref_R);
            SERIAL_ECHOPGM(" A:"); SERIAL_ECHO(ref_A);
            SERIAL_ECHOPGM(" Z:"); SERIAL_ECHOLN(ref_Z);
            SERIAL_ECHOLNPGM("=======================================");
            return;
        } else {
            SERIAL_ECHOLNPGM("CHYBA: Zadejte platne cislo hnizda (M667 C0, C1 nebo C2).");
            return;
        }
    }

    // --- S) AUTO-SCAN ZÁSOBNÍKU ---
    if (parser.seen('S')) {
        if (!is_atc_calibrated) {
            SERIAL_ECHOLNPGM("CHYBA: ATC neni zkalibrovano! Najedte rucne do hnizda a zadejte napr: M667 C1");
            return;
        }

        SERIAL_ECHOLNPGM(">>> Zahajuji AUTO-SCAN zasobniku...");

        // Ochrana proti relativnímu pohybu
        bool was_relative = GcodeSuite::axis_is_relative(X_AXIS);
        if (was_relative) GcodeSuite::set_relative_mode(false);

        // 1. Vymazání staré paměti (chceme absolutně čistou inventuru)
        for (int i = 0; i < 3; i++) dock_tool[i] = -1;
        
        // 2. Kontrola a případné odložení nástroje v ruce
        uint8_t head_id = read_pogo_pins();
        if (head_id != HEAD_EMPTY && head_id != HEAD_ERROR) {
            tool_in_hand = head_id;
            SERIAL_ECHOLNPGM("POZOR: Ruka neni prazdna. Automaticky odkladam nastroj pred skenovanim...");
            
            // Robot nástroj odloží a funkce rovnou zapíše jeho ID do příslušného dock_tool[i]
            if (!unload_current_tool()) {
                SERIAL_ECHOLNPGM("FATAL: Nelze odlozit nastroj, Auto-Scan prerusen!");
                return; 
            }
        } else {
            tool_in_hand = -1;
        }

        float current_A = DEGREES(atan2(current_position.y, current_position.x));
        float current_R = hypot(current_position.x, current_position.y);
        exec_move("G8", current_A, current_R, ref_Z + 100);

        // 3. Výpočet tras a seřazení (Optimalizace)
        float cur_A = DEGREES(atan2(current_position.y, current_position.x));
        int scan_order[3] = {0, 1, 2};
        float dist[3];
        
        for (int i = 0; i < 3; i++) {
            float dock_A = ref_A + magazine[i].a_offset;
            dist[i] = fabs(cur_A - dock_A);
            if (dist[i] > 180.0) dist[i] = 360.0 - dist[i];
        }
        
        for (int i = 0; i < 2; i++) {
            for (int j = i + 1; j < 3; j++) {
                if (dist[scan_order[i]] > dist[scan_order[j]]) {
                    int temp = scan_order[i];
                    scan_order[i] = scan_order[j];
                    scan_order[j] = temp;
                }
            }
        }

        // 4. Samotný fyzický sken v optimalizovaném pořadí
        for (int step = 0; step < 3; step++) {
            int i = scan_order[step];
            
            if (read_dock_sensor(i) == 1) { 
                // OPRAVA: Skenujeme fyzicky JEN pokud o tomto hnízdě ještě nic nevíme
                if (dock_tool[i] == -1) {
                    SERIAL_ECHOPGM("Dock "); SERIAL_ECHO(i); SERIAL_ECHOLNPGM(" je plny. Skenuji ID...");
                    
                    float t_A = ref_A + magazine[i].a_offset;
                    
                    exec_move("G9", t_A, ref_R, ref_Z + 100); 
                    gcode.process_subcommands_now((char *)"M280 P0 S180"); 
                    
                    exec_move("G9", t_A, ref_R, ref_Z, 1000); 
                    
                    safe_delay(500); 
                    gcode.process_subcommands_now((char *)"M280 P0 S20"); 
                    safe_delay(1000); 
                    
                    uint8_t scanned_id = read_pogo_pins();
                    
                    gcode.process_subcommands_now((char *)"M280 P0 S180"); 
                    safe_delay(500);
                    
                    if (scanned_id != HEAD_EMPTY && scanned_id != HEAD_ERROR) {
                        dock_tool[i] = scanned_id;
                        SERIAL_ECHOPGM(">>> Nalezen nastroj T"); SERIAL_ECHOLN(scanned_id);
                    } else {
                        dock_tool[i] = -1;
                        SERIAL_ECHOLNPGM("!!! Chyba cteni! POGO piny neprecetly platne ID.");
                    }
                    
                    exec_move("G9", t_A, ref_R, ref_Z + 60, 1500); 
                } else {
                    // Robot už ví, co tu je, protože to sem před chvílí položil
                    SERIAL_ECHOPGM("Dock "); SERIAL_ECHO(i); SERIAL_ECHOPGM(" preskocen (Nastroj T"); 
                    SERIAL_ECHO(dock_tool[i]); SERIAL_ECHOLNPGM(" sem byl prave odlozen).");
                }
            } else { 
                // Pokud senzor říká, že je hnízdo prázdné
                dock_tool[i] = -1; 
            }
        }
        SERIAL_ECHOLNPGM(">>> AUTO-SCAN DOKONCEN!");
        return; 

        if (was_relative) GcodeSuite::set_relative_mode(true);
        return;
    }

    // --- U) UNLOAD: POUZE ODLOŽENÍ NÁSTROJE ---
    if (parser.seen('U')) {
        if (!is_atc_calibrated) {
            SERIAL_ECHOLNPGM("CHYBA: ATC neni zkalibrovano! Najedte rucne do hnizda a zadejte napr: M667 C1");
            return;
        }
        if (tool_in_hand == -1) {
            SERIAL_ECHOLNPGM("Ruka je prazdna, neni co odlozit.");
            return;
        }

        // OPTIMALIZACE: Voláme naši novou funkci
        if (unload_current_tool()) {
            SERIAL_ECHOLNPGM(">> Nastroj uspesne odlozen. Ruka je prazdna.");
        }
        return;
    }

    // --- ZBYTEK M667 (Ruční zápis) ---
    if (parser.seen('D') && parser.seen('T')) {
        int d = parser.intval('D');
        int t = parser.intval('T');
        if (d >= 0 && d < 3) dock_tool[d] = t;
    }
    if (parser.seen('H')) tool_in_hand = parser.intval('H');

    // --- DIAGNOSTIKA ---
    uint8_t head_id = read_pogo_pins();
    SERIAL_ECHOLNPGM("=======================================");
    SERIAL_ECHOLNPGM("          DIAGNOSTIKA ATC              ");
    SERIAL_ECHOLNPGM("=======================================");
    SERIAL_ECHOPGM("Kalibrace (Teach-in): "); SERIAL_ECHOLNPGM(is_atc_calibrated ? "HOTOVO" : "CHYBI !!!");
    SERIAL_ECHOLNPGM("---------------------------------------");
    SERIAL_ECHOPGM("[HLAVA] POGO: "); print_binary_4bit(head_id); SERIAL_ECHOPGM(" -> ID: "); SERIAL_ECHO(head_id);
    SERIAL_ECHOPGM("\nStav: ");
    if (head_id == HEAD_EMPTY) SERIAL_ECHOLNPGM("PRAZDNA (1111)");
    else if (head_id == HEAD_ERROR) SERIAL_ECHOLNPGM("CHYBA (0000)");
    else { 
        SERIAL_ECHOPGM("UPNUTO: T"); SERIAL_ECHO(head_id); 
        if (tool_table[head_id].name[0] != '\0') {
            SERIAL_ECHOPGM(" ("); SERIAL_ECHO(tool_table[head_id].name); SERIAL_ECHOPGM(")");
        }
        SERIAL_EOL();
    }
    SERIAL_ECHOLNPGM("---------------------------------------");
    SERIAL_ECHOLNPGM("[ZASOBNIK]");
    for (int i = 0; i < 3; i++) {
        SERIAL_ECHOPGM("D"); SERIAL_ECHO(i);
        SERIAL_ECHOPGM(read_dock_sensor(i) == 1 ? " | PLNO " : " | PRAZD");
        SERIAL_ECHOPGM(" | Pamet: ");
        if (dock_tool[i] == -1) SERIAL_ECHOLNPGM("NIC"); 
        else { 
            SERIAL_ECHOPGM("T"); SERIAL_ECHO(dock_tool[i]); 
            if (tool_table[dock_tool[i]].name[0] != '\0') {
                SERIAL_ECHOPGM(" ("); SERIAL_ECHO(tool_table[dock_tool[i]].name); SERIAL_ECHOPGM(")");
            }
            SERIAL_EOL();
        }
    }
    SERIAL_ECHOLNPGM("=======================================");
}

// ==========================================
// 6. M6: AUTOMATICKÁ VÝMĚNA NÁSTROJE
// ==========================================
void GcodeSuite::M6() {
    if (!parser.seenval('T')) return;

    if (!is_atc_calibrated) {
        SERIAL_ECHOLNPGM("CHYBA: ATC neni zkalibrovano! Najedte rucne do hnizda a zadejte napr: M667 C1");
        return;
    }

    // Ochrana proti relativnímu pohybu
    bool was_relative = GcodeSuite::axis_is_relative(X_AXIS);
    if (was_relative) GcodeSuite::set_relative_mode(false);

    int8_t target_tool = parser.value_int();
    if (tool_in_hand == target_tool) return;

    int8_t source_dock = -1;
    for (int i = 0; i < 3; i++) { if (dock_tool[i] == target_tool) { source_dock = i; break; } }

    if (source_dock == -1) {
        SERIAL_ECHOLNPGM("CHYBA: Nastroj neni v pameti zasobniku!"); return;
    }

    // --- FÁZE 1: ODLOŽENÍ ---
    // OPTIMALIZACE: Místo hromady kódu jen zavoláme naši novou funkci
    if (!unload_current_tool()) {
        return; // Pokud se nepodařilo odložit (např. chybí volný dock), přerušíme celou výměnu
    }

    // --- FÁZE 2: VYZVEDNUTÍ ---
    if (read_dock_sensor(source_dock) == 0) { 
        SERIAL_ECHOLNPGM("FATAL: Cilovy dock je fyzicky prazdny!"); 
        dock_tool[source_dock] = -1; return; 
    }

    float t_A = ref_A + magazine[source_dock].a_offset;

    SERIAL_ECHOPGM(">> Vyzvedavam z D"); SERIAL_ECHOLN(source_dock);

    exec_move("G8", t_A, ref_R,       ref_Z + 60);       
    gcode.process_subcommands_now((char *)"M280 P0 S180"); 
    
    exec_move("G9", t_A, ref_R,       ref_Z, 1000);       
    
    safe_delay(1000); 
    
    // Zjistíme úhel z tabulky. Pokud tam uživatel nic nezadal (je tam 0 nebo blbost), 
    // dáme bezpečnou výchozí hodnotu 20.
    int clamp = tool_table[target_tool].clamp_angle;
    if (clamp <= 0 || clamp > 180) clamp = 20; 

    // Poskládáme příkaz s vypočítaným úhlem
    char cmd_servo[32];
    sprintf(cmd_servo, "M280 P0 S%d", clamp);
    gcode.process_subcommands_now(cmd_servo); 
    
    safe_delay(1000);
    
    if (read_pogo_pins() != target_tool) {
        SERIAL_ECHOLNPGM("!!! FATAL ERROR: Neshoda ID nastroje !!!");
        gcode.process_subcommands_now((char *)"M280 P0 S180"); return; 
    }

    exec_move("G9", t_A, ref_R + 60,  ref_Z + 6, 1500);   
    exec_move("G8", t_A, ref_R + 100, ref_Z + 100);       

    tool_in_hand = target_tool;
    dock_tool[source_dock] = -1; 
    SERIAL_ECHOLNPGM(">> ATC Hotovo.");

    if (was_relative) GcodeSuite::set_relative_mode(true);
    return;
}