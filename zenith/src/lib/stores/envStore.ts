// Wrapper for environment variables fetched from Tauri backend.

import { get_env } from '$lib/utils';
import { asyncReadable, type Loadable } from '@square/svelte-store';

export type EnvStore = {
	launchDir: string;
};

const defaultEnv: EnvStore = {
	launchDir: '../launches'
};

/**
 * Store for environment variables fetched from Tauri (Rust) backend.
 */
export const envStore: Loadable<EnvStore> = asyncReadable(defaultEnv, async () => {
	const launchDir = await get_env('LAUNCH_DIR', '../launches');
	return { launchDir };
});
