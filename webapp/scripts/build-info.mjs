#!/usr/bin/env node
/**
 * Writes dist/build-info.json so a running container can say which build it is.
 *
 * The commit is looked up in this order, because the container is built from a
 * copy of `webapp/` that carries no `.git`:
 *   BUILD_COMMIT → `.build-commit` written by deploy/bob/sync.sh → local git.
 */
import { execFileSync } from 'node:child_process';
import { readFileSync, writeFileSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const read = (file) => readFileSync(path.join(root, file), 'utf8').trim();

function fromGit() {
  const git = (...args) => execFileSync('git', args, { cwd: root, stdio: ['ignore', 'pipe', 'ignore'] }).toString().trim();
  const commit = git('rev-parse', '--short', 'HEAD');
  return git('status', '--porcelain') ? `${commit}-dirty` : commit;
}

function commit() {
  for (const source of [() => process.env.BUILD_COMMIT, () => read('.build-commit'), fromGit]) {
    try {
      const value = source();
      if (value) return value.slice(0, 40);
    } catch { /* Try the next source; an unknown commit must not fail a build. */ }
  }
  return null;
}

const { version } = JSON.parse(read('package.json'));
const info = { version, builtAt: new Date().toISOString(), commit: commit() };
writeFileSync(path.join(root, 'dist', 'build-info.json'), `${JSON.stringify(info)}\n`, 'utf8');
console.log(`build-info: ${info.version} ${info.commit ?? 'no-commit'} ${info.builtAt}`);
