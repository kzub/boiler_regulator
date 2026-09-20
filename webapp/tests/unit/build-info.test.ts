import { mkdtemp, writeFile } from 'node:fs/promises';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import { describe, expect, it } from 'vitest';
import { loadBuildInfo } from '../../src/server/build-info.js';

async function stampFile(content: string): Promise<string> {
  const file = join(await mkdtemp(join(tmpdir(), 'boiler-build-')), 'build-info.json');
  await writeFile(file, content);
  return file;
}

describe('build info', () => {
  it('reports the stamp written by a build', async () => {
    const file = await stampFile(JSON.stringify({ version: '0.1.0', builtAt: '2026-09-20T09:45:48.297Z', commit: 'abc1234' }));
    expect(loadBuildInfo(file)).toEqual({ version: '0.1.0', builtAt: '2026-09-20T09:45:48.297Z', commit: 'abc1234' });
  });

  it('reports a source run when nothing was built', async () => {
    expect(loadBuildInfo(join(tmpdir(), 'missing-build-info.json'))).toEqual({ version: 'dev', builtAt: null, commit: null });
  });

  it('ignores a malformed stamp instead of failing a request', async () => {
    expect(loadBuildInfo(await stampFile('{"version":42}'))).toEqual({ version: 'dev', builtAt: null, commit: null });
  });
});
