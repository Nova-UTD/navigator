/* eslint-disable */
import { execa } from 'execa';
import fs from 'fs';

const OUT_BINARY_NAME = 'ros2-querier';
const OUT_DIR = 'src-tauri/bin/';

// Append exe if on Windows.
let extension = '';
if (process.platform === 'win32') {
	extension = '.exe';
}

// Build the Python binary using PyInstaller.
async function build() {
	return execa('pyinstaller', [
		'--onefile',
		'shim/main.py',
		'--name',
		OUT_BINARY_NAME,
		'--workpath',
		'build-py',
		'--distpath',
		OUT_DIR,
		'--specpath',
		'spec/',
		'--clean'
	]);
}

// Append the target triple to the binary name.
async function appendTargetTriple() {
	// Use rustc to determine the target triple.
	const rustInfo = (await execa('rustc', ['-vV'])).stdout;
	const targetTriple = /host: (\S+)/g.exec(rustInfo)[1];
	if (!targetTriple) {
		throw new Error('failed to determine platform target triple');
	}

	// Rename the binary to include the target triple.
	const pathWithExtension = `${OUT_DIR}/${OUT_BINARY_NAME}${extension}`;
	const pathWithTargetTriple = `${OUT_DIR}/${OUT_BINARY_NAME}-${targetTriple}${extension}`;
	try {
		fs.renameSync(pathWithExtension, pathWithTargetTriple);
	} catch (e) {
		throw new Error(`failed to rename ${pathWithExtension} -> ${pathWithTargetTriple}`);
	}

	return pathWithTargetTriple;
}

async function main() {
	await build()
		.then(() => {
			console.log(`✅ Successfully built ${OUT_DIR}/${OUT_BINARY_NAME}`);
		})
		.catch((e) => {
			throw new Error(`error while building: ${e.message}`);
		});

	await appendTargetTriple()
		.then((path) => {
			console.log(`✅ Successfully appended target triple to ${path}`);
		})
		.catch((e) => {
			throw new Error(`error while appending target triple: ${e.message}`);
		});
}

main().catch((e) => {
	console.error(`❌ Error during Python build: ${e.message}`);
});
