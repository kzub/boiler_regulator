import { readFileSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import type { BuildInfo } from '../shared/contracts.js';

/** Running from sources: nothing was built, so there is no stamp to report. */
const unbuilt: BuildInfo = { version: 'dev', builtAt: null, commit: null };

const defaultPath = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '../build-info.json');

/** Reads the stamp `scripts/build-info.mjs` writes next to the compiled server. */
export function loadBuildInfo(file: string = defaultPath): BuildInfo {
  try {
    const parsed: unknown = JSON.parse(readFileSync(file, 'utf8'));
    if (!parsed || typeof parsed !== 'object') return unbuilt;
    const info = parsed as Record<string, unknown>;
    return {
      version: typeof info.version === 'string' ? info.version : unbuilt.version,
      builtAt: typeof info.builtAt === 'string' ? info.builtAt : null,
      commit: typeof info.commit === 'string' ? info.commit : null,
    };
  } catch { return unbuilt; }
}
