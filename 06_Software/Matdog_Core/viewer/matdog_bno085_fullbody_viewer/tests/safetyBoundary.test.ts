/**
 * Mechanical enforcement of the task's hard safety boundary.
 *
 * The viewer is a read-only host-side program. These tests scan the whole
 * viewer source tree so that a future edit cannot quietly introduce a write
 * path towards the ESP32, the BNO085 DCD, the ST3215 bus or the battery
 * system without failing the suite.
 */

import { readdirSync, readFileSync, statSync } from 'node:fs';
import { extname, join, relative } from 'node:path';
import { describe, expect, it } from 'vitest';

import { VIEWER_DIR } from './support/nodeEnvironment';

/**
 * Directories holding shipped viewer code. The test files themselves are not
 * scanned: they necessarily quote the forbidden patterns in order to check
 * for them.
 */
const SCANNED_DIRECTORIES = ['src', 'scripts'];
const SCANNED_ROOT_FILES = ['index.html', 'vite.config.ts'];
const SCANNED_EXTENSIONS = new Set(['.ts', '.js', '.mjs', '.html', '.css']);

function collectSourceFiles(): string[] {
  const files: string[] = [];
  const walk = (directory: string): void => {
    for (const entry of readdirSync(directory)) {
      const full = join(directory, entry);
      if (statSync(full).isDirectory()) {
        walk(full);
      } else if (SCANNED_EXTENSIONS.has(extname(entry))) {
        files.push(full);
      }
    }
  };
  for (const directory of SCANNED_DIRECTORIES) walk(join(VIEWER_DIR, directory));
  for (const file of SCANNED_ROOT_FILES) files.push(join(VIEWER_DIR, file));
  return files;
}

const SOURCE_FILES = collectSourceFiles();

function read(file: string): string {
  return readFileSync(file, 'utf8');
}

/**
 * Patterns that must not appear as executable code. Occurrences inside
 * comments are allowed, because the safety boundary itself is documented in
 * comments throughout the viewer.
 */
function codeLines(file: string): { line: string; number: number }[] {
  const result: { line: string; number: number }[] = [];
  let inBlockComment = false;
  read(file)
    .split('\n')
    .forEach((rawLine, index) => {
      let line = rawLine;
      if (inBlockComment) {
        const end = line.indexOf('*/');
        if (end === -1) return;
        line = line.slice(end + 2);
        inBlockComment = false;
      }
      const blockStart = line.indexOf('/*');
      if (blockStart !== -1) {
        const end = line.indexOf('*/', blockStart + 2);
        if (end === -1) {
          inBlockComment = true;
          line = line.slice(0, blockStart);
        } else {
          line = line.slice(0, blockStart) + line.slice(end + 2);
        }
      }
      const lineComment = line.indexOf('//');
      if (lineComment !== -1) line = line.slice(0, lineComment);
      const htmlComment = line.indexOf('<!--');
      if (htmlComment !== -1) line = line.slice(0, htmlComment);
      if (line.trim().length > 0) result.push({ line, number: index + 1 });
    });
  return result;
}

describe('safety boundary', () => {
  it('scans a non-trivial number of viewer source files', () => {
    expect(SOURCE_FILES.length).toBeGreaterThan(10);
  });

  it('contains no serial write path at all', () => {
    const forbidden = [
      /\.writable\b/,
      /getWriter\s*\(/,
      /\bWritableStream\b/,
      /TextEncoderStream/,
      /\bnew TextEncoder\b/,
      /port\.write/,
    ];
    for (const file of SOURCE_FILES) {
      for (const { line, number } of codeLines(file)) {
        for (const pattern of forbidden) {
          expect(
            pattern.test(line),
            `${relative(VIEWER_DIR, file)}:${number} introduces a serial write path: ${line.trim()}`,
          ).toBe(false);
        }
      }
    }
  });

  it('never emits a firmware, DCD or calibration command', () => {
    // Command vocabulary of matdog_bno085_dcd_phase_c3.ino plus DCD/calibration verbs.
    const forbidden = [
      /['"`]\s*SAVE\s*['"`]/,
      /['"`]\s*STATUS\s*['"`]/,
      /\bDCD_SAVE\b/,
      /sh2_setDcd/i,
      /setCalConfig/i,
      /\besptool\b/i,
      /arduino-cli/i,
      /\bavrdude\b/i,
    ];
    for (const file of SOURCE_FILES) {
      for (const { line, number } of codeLines(file)) {
        for (const pattern of forbidden) {
          expect(
            pattern.test(line),
            `${relative(VIEWER_DIR, file)}:${number} looks like a device command: ${line.trim()}`,
          ).toBe(false);
        }
      }
    }
  });

  it('never touches the servo bus, torque, EEPROM or the battery system', () => {
    const forbidden = [
      /\bst3215\b/i,
      /\bfeetech\b/i,
      /torque/i,
      /\beeprom\b/i,
      /\bdaly\b/i,
      /\bbms\b/i,
      /goal_?position/i,
      /\bservo(Bus|Write|Command|Id)\b/i,
    ];
    for (const file of SOURCE_FILES) {
      for (const { line, number } of codeLines(file)) {
        for (const pattern of forbidden) {
          expect(
            pattern.test(line),
            `${relative(VIEWER_DIR, file)}:${number} references the servo/battery stack: ${line.trim()}`,
          ).toBe(false);
        }
      }
    }
  });

  it('hard-codes no tty device path', () => {
    const forbidden = [/\/dev\/tty/, /\/dev\/serial\//, /usb-Espressif/i, /COM\d+['"`]/];
    for (const file of SOURCE_FILES) {
      for (const { line, number } of codeLines(file)) {
        for (const pattern of forbidden) {
          expect(
            pattern.test(line),
            `${relative(VIEWER_DIR, file)}:${number} hard-codes a serial device: ${line.trim()}`,
          ).toBe(false);
        }
      }
    }
  });

  it('never writes outside the viewer directory', () => {
    // Only the staging script writes anything, and only into public/canonical.
    const writeApis = /\b(writeFileSync|writeFile|appendFile|rmSync|unlinkSync|mkdirSync|copyFileSync|cpSync)\b/;
    for (const file of SOURCE_FILES) {
      const relativePath = relative(VIEWER_DIR, file);
      for (const { line, number } of codeLines(file)) {
        if (!writeApis.test(line)) continue;
        expect(
          relativePath,
          `${relativePath}:${number} performs filesystem writes outside the staging script`,
        ).toBe(join('scripts', 'sync_canonical_assets.mjs'));
      }
    }
  });

  it('only ever opens the serial port for reading', () => {
    const serialFile = join(VIEWER_DIR, 'src/serial/webSerial.ts');
    const executable = codeLines(serialFile)
      .map((entry) => entry.line)
      .join('\n');
    expect(executable).toContain('getReader()');
    expect(executable).not.toContain('getWriter');
    // The only mentions of the writable side are in the safety documentation.
    expect(executable).not.toContain('.writable');
    expect(read(serialFile)).toContain('is never touched');
  });

  it('uses SH2_ROTATION_VECTOR (RV) and never GAME_ROTATION_VECTOR as orientation', () => {
    for (const file of SOURCE_FILES) {
      for (const { line, number } of codeLines(file)) {
        expect(
          /GAME_ROTATION_VECTOR|\bGAME\b/.test(line),
          `${relative(VIEWER_DIR, file)}:${number} uses the game rotation vector: ${line.trim()}`,
        ).toBe(false);
      }
    }
    const parserSource = read(join(VIEWER_DIR, 'src/serial/lineParser.ts'));
    expect(parserSource).toContain("case 'RV'");
  });
});
